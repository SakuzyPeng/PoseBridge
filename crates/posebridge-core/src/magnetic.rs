//! Exclusive magnetic monitoring/calibration. One parser owns all register replies.
use super::{SharedRef, lock};
use crate::model::{decimal, optional_decimal};
use crate::protocol::{self, Frame, Parser};
use crate::transport::{self, Connection, Reader, Writer};
use crate::{
    ConnectionState, DeviceCommand, Error, OperationOutcome, OperationStatus, Result, Source,
    new_id,
};
use serde::{Deserialize, Serialize};
use std::collections::VecDeque;
use std::time::{Duration, Instant};
use tokio::sync::{mpsc, watch};

pub const MAGNETIC_HISTORY_CAPACITY: usize = 256;
const POLL_INTERVAL: Duration = Duration::from_millis(200);
const FRESH_FOR: Duration = Duration::from_millis(500);

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MagneticCursor {
    #[serde(with = "cursor_decimal")]
    pub instance_id: u64,
    #[serde(with = "cursor_decimal")]
    pub session_id: u64,
    #[serde(with = "cursor_decimal")]
    pub sequence: u64,
}

mod cursor_decimal {
    pub use crate::model::decimal::serialize;
    use serde::{Deserialize, Deserializer, de::Error};
    pub fn deserialize<'de, D: Deserializer<'de>>(d: D) -> Result<u64, D::Error> {
        let text = String::deserialize(d)?;
        if text.is_empty() || !text.bytes().all(|c| c.is_ascii_digit()) {
            return Err(D::Error::custom("cursor values must be decimal strings"));
        }
        text.parse::<u64>()
            .ok()
            .filter(|v| *v <= i64::MAX as u64)
            .ok_or_else(|| D::Error::custom("cursor exceeds positive int64 range"))
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum MagneticPhase {
    #[default]
    Idle,
    Opening,
    Monitoring,
    ExternalCalibration,
    Starting,
    Calibrating,
    Stopping,
    Calibrated,
    Saving,
    Closed,
    Failed,
}

#[derive(Clone, Debug, Serialize)]
pub struct MagneticSample {
    pub cursor: MagneticCursor,
    #[serde(with = "decimal")]
    pub window_id: u64,
    pub phase: MagneticPhase,
    /// Sensor axes; device register output, not necessarily uncompensated ADC data.
    pub register_xyz: [i16; 3],
    pub field_ut: Option<[f64; 3]>,
    #[serde(with = "decimal")]
    pub received_ns: u64,
    #[serde(with = "decimal")]
    pub age_ns: u64,
    pub fresh: bool,
}

#[derive(Clone, Debug, Serialize)]
pub struct MagneticStatistics {
    #[serde(with = "decimal")]
    pub window_id: u64,
    #[serde(with = "decimal")]
    pub sample_count: u64,
    #[serde(with = "decimal")]
    pub elapsed_ns: u64,
    pub frozen: bool,
    pub actual_rate_hz: f64,
    pub minimum_counts: Option<[i16; 3]>,
    pub maximum_counts: Option<[i16; 3]>,
    pub span_counts: Option<[i32; 3]>,
}

/// A non-consuming, atomic query. All ages use the same query instant.
#[derive(Clone, Debug, Serialize)]
pub struct MagneticBatch {
    pub schema: u32,
    pub source_id: String,
    #[serde(with = "decimal")]
    pub instance_id: u64,
    #[serde(with = "decimal")]
    pub session_id: u64,
    pub cursor: Option<MagneticCursor>,
    pub reset: bool,
    #[serde(with = "decimal")]
    pub history_overrun: u64,
    /// Magnetic reception is fresh, independent of pose freshness.
    pub active: bool,
    pub phase: MagneticPhase,
    pub sensor_type: Option<u16>,
    pub scale_ut_per_count: Option<f64>,
    pub type_error: Option<String>,
    pub calsw: Option<u16>,
    #[serde(with = "optional_decimal")]
    pub calsw_age_ns: Option<u64>,
    pub device_may_be_calibrating: bool,
    pub latest: Option<MagneticSample>,
    pub samples: Vec<MagneticSample>,
    pub statistics: MagneticStatistics,
    pub operation: Option<OperationStatus>,
    /// Separate from the requested operation, including when that operation was cancelled.
    pub cleanup: Option<OperationStatus>,
    pub last_error: Option<String>,
}

fn ns(d: Duration) -> u64 {
    d.as_nanos().min(i64::MAX as u128) as u64
}

// WIT SDK DipSensorMagHelper.cs at 9efaab0fdd6a06dc807bf80402e58aa91b431c6f.
// Its unknown-type identity fallback does not establish physical units.
fn scale(sensor_type: u16) -> Option<f64> {
    match sensor_type {
        2 => Some(0.15),
        3 => Some(0.013),
        4 => Some(0.058),
        5 => Some(0.098),
        6 => Some(1.0 / 120.0),
        7 => Some(0.02),
        _ => None,
    }
}

struct Window {
    id: u64,
    count: u64,
    first: Option<Instant>,
    last: Option<Instant>,
    end: Option<Instant>,
    minimum: Option<[i16; 3]>,
    maximum: Option<[i16; 3]>,
}
impl Default for Window {
    fn default() -> Self {
        Self {
            id: new_id(),
            count: 0,
            first: None,
            last: None,
            end: None,
            minimum: None,
            maximum: None,
        }
    }
}
impl Window {
    fn observe(&mut self, xyz: [i16; 3], now: Instant) {
        if self.end.is_some() {
            return;
        }
        self.count += 1;
        self.first.get_or_insert(now);
        self.last = Some(now);
        self.minimum = Some(
            self.minimum
                .map_or(xyz, |v| std::array::from_fn(|i| v[i].min(xyz[i]))),
        );
        self.maximum = Some(
            self.maximum
                .map_or(xyz, |v| std::array::from_fn(|i| v[i].max(xyz[i]))),
        );
    }
    fn snapshot(&self, now: Instant) -> MagneticStatistics {
        let elapsed = self.first.map_or(Duration::ZERO, |first| {
            self.end.unwrap_or(now).saturating_duration_since(first)
        });
        let span = self
            .first
            .zip(self.last)
            .map_or(0.0, |(a, b)| b.duration_since(a).as_secs_f64());
        MagneticStatistics {
            window_id: self.id,
            sample_count: self.count,
            elapsed_ns: ns(elapsed),
            frozen: self.end.is_some(),
            actual_rate_hz: if span > 0.0 {
                self.count.saturating_sub(1) as f64 / span
            } else {
                0.0
            },
            minimum_counts: self.minimum,
            maximum_counts: self.maximum,
            span_counts: self
                .minimum
                .zip(self.maximum)
                .map(|(a, b)| std::array::from_fn(|i| i32::from(b[i]) - i32::from(a[i]))),
        }
    }
}

#[derive(Default)]
pub(super) struct State {
    instance_id: u64,
    session_id: u64,
    sequence: u64,
    pub phase: MagneticPhase,
    running: bool,
    sensor_type: Option<u16>,
    type_error: Option<String>,
    calsw: Option<u16>,
    calsw_at: Option<Instant>,
    /// Set BEFORE the start write, cleared only after an observed exit.
    owned: bool,
    records: VecDeque<(Instant, MagneticSample)>,
    window: Window,
    pub cleanup: Option<OperationStatus>,
    pub last_error: Option<String>,
}
impl State {
    pub fn new(instance_id: u64, session_id: u64) -> Self {
        Self {
            instance_id,
            session_id,
            phase: MagneticPhase::Opening,
            running: true,
            ..Self::default()
        }
    }
    pub fn ready(&self) -> bool {
        self.running
            && !self.records.is_empty()
            && !matches!(
                self.phase,
                MagneticPhase::Opening | MagneticPhase::Closed | MagneticPhase::Failed
            )
    }
    fn observe(&mut self, xyz: [i16; 3], received: Instant, start: Instant) {
        self.sequence += 1;
        self.window.observe(xyz, received);
        let sample = MagneticSample {
            cursor: MagneticCursor {
                instance_id: self.instance_id,
                session_id: self.session_id,
                sequence: self.sequence,
            },
            window_id: self.window.id,
            phase: self.phase,
            register_xyz: xyz,
            field_ut: self
                .sensor_type
                .and_then(scale)
                .map(|scale| xyz.map(|v| f64::from(v) * scale)),
            received_ns: ns(received.saturating_duration_since(start)),
            age_ns: 0,
            fresh: true,
        };
        if self.records.len() == MAGNETIC_HISTORY_CAPACITY {
            self.records.pop_front();
        }
        self.records.push_back((received, sample));
    }
    fn observe_calsw(&mut self, value: u16, now: Instant) {
        self.calsw = Some(value);
        self.calsw_at = Some(now);
        match self.phase {
            MagneticPhase::Monitoring
            | MagneticPhase::ExternalCalibration
            | MagneticPhase::Calibrated
                if value == 7 =>
            {
                self.phase = MagneticPhase::ExternalCalibration
            }
            MagneticPhase::ExternalCalibration if value == 0 => {
                self.phase = MagneticPhase::Monitoring
            }
            MagneticPhase::Calibrating if value == 0 => {
                self.phase = MagneticPhase::Calibrated;
                self.owned = false;
                self.window.end.get_or_insert(now);
            }
            _ => {}
        }
    }
    pub fn finish(&mut self, error: Option<String>) {
        self.running = false;
        self.window.end.get_or_insert(Instant::now());
        self.phase = if error.is_some() {
            MagneticPhase::Failed
        } else {
            MagneticPhase::Closed
        };
        self.last_error = error;
    }
    pub fn since(
        &self,
        cursor: Option<MagneticCursor>,
        source_id: String,
        operation: Option<OperationStatus>,
        now: Instant,
    ) -> MagneticBatch {
        let reset = cursor.is_some_and(|c| {
            c.instance_id != self.instance_id
                || c.session_id != self.session_id
                || c.sequence > self.sequence
        });
        let after = if reset {
            0
        } else {
            cursor.map_or(0, |c| c.sequence)
        };
        let active = self.running
            && self.last_error.is_none()
            && self
                .records
                .back()
                .is_some_and(|(t, _)| now.saturating_duration_since(*t) < FRESH_FOR);
        let at_query = |(received, sample): &(Instant, MagneticSample)| {
            let mut sample = sample.clone();
            sample.age_ns = ns(now.saturating_duration_since(*received));
            sample.fresh = active && sample.age_ns < ns(FRESH_FOR);
            sample
        };
        MagneticBatch {
            schema: 1,
            source_id,
            instance_id: self.instance_id,
            session_id: self.session_id,
            cursor: self.records.back().map(|(_, s)| s.cursor),
            reset,
            history_overrun: if cursor.is_some() && !reset {
                self.records.front().map_or(0, |(_, s)| {
                    s.cursor.sequence.saturating_sub(after.saturating_add(1))
                })
            } else {
                0
            },
            active,
            phase: self.phase,
            sensor_type: self.sensor_type,
            scale_ut_per_count: self.sensor_type.and_then(scale),
            type_error: self.type_error.clone(),
            calsw: self.calsw,
            calsw_age_ns: self.calsw_at.map(|t| ns(now.saturating_duration_since(t))),
            device_may_be_calibrating: self.owned || self.calsw == Some(7),
            latest: self.records.back().map(at_query),
            samples: self
                .records
                .iter()
                .filter(|(_, s)| s.cursor.sequence > after)
                .map(at_query)
                .collect(),
            statistics: self.window.snapshot(now),
            operation,
            cleanup: self.cleanup.clone(),
            last_error: self.last_error.clone(),
        }
    }
}

struct Io<'a> {
    reader: Reader<'a>,
    writer: Writer<'a>,
    parser: Parser,
    shared: SharedRef,
    cancel: watch::Receiver<bool>,
    start: Instant,
}
impl Io<'_> {
    async fn receive(&mut self) -> Result<Vec<Frame>> {
        let bytes = self.reader.read().await?;
        let at = Instant::now();
        let discarded = self.parser.discarded_bytes;
        let invalid = self.parser.invalid_frames;
        let frames = self.parser.push(&bytes);
        let mut s = lock(&self.shared);
        s.status.bytes_received += bytes.len() as u64;
        s.status.frames_received += frames.len() as u64;
        s.status.discarded_bytes += self.parser.discarded_bytes - discarded;
        s.status.invalid_frames += self.parser.invalid_frames - invalid;
        for frame in &frames {
            match frame {
                Frame::Registers {
                    address: 0x3a,
                    values,
                } => s
                    .magnetic
                    .observe([values[0], values[1], values[2]], at, self.start),
                Frame::Registers { address: 1, values } => {
                    s.magnetic.observe_calsw(values[0] as u16, at)
                }
                _ => {}
            }
        }
        Ok(frames)
    }
    async fn write(&mut self, bytes: &[u8]) -> Result<()> {
        transport::cancel_after(
            &mut self.cancel,
            Duration::from_secs(2),
            self.writer.write(bytes),
        )
        .await
    }
    async fn read_register(&mut self, address: u16, expected: Option<u16>) -> Result<[u16; 8]> {
        let mut cancel = self.cancel.clone();
        transport::cancel_after(&mut cancel, Duration::from_secs(3), async {
            self.write(&protocol::read_register(address)).await?;
            let mut retry = Instant::now() + Duration::from_millis(250);
            loop {
                tokio::select! {
                    frames = self.receive() => {
                        for frame in frames? {
                            if let Frame::Registers { address: base, values } = frame && base == address {
                                let values = values.map(|v| v as u16);
                                if expected.is_none_or(|v| values[0] == v) { return Ok(values); }
                            }
                        }
                    }
                    _ = tokio::time::sleep_until(retry.into()) => {
                        // Only control readback retries. A magnetic request remains singly outstanding.
                        if address != 0x3a { self.write(&protocol::read_register(address)).await?; }
                        retry = Instant::now() + Duration::from_millis(250);
                    }
                }
            }
        }).await.map_err(|e| match e {
            Error::Timeout(_) => Error::Timeout(format!("magnetic register 0x{address:02x}, expected {expected:?}")), other => other,
        })
    }
    async fn delay(&mut self, duration: Duration) -> Result<()> {
        let mut cancel = self.cancel.clone();
        transport::cancel_after(&mut cancel, duration + Duration::from_secs(1), async {
            let deadline = Instant::now() + duration;
            loop {
                tokio::select! {
                    biased;
                    _ = tokio::time::sleep_until(deadline.into()) => return Ok(()),
                    frames = self.receive() => { frames?; }
                }
            }
        })
        .await
    }
    fn operation(&self, cleanup: bool, change: impl FnOnce(&mut OperationStatus)) {
        let mut s = lock(&self.shared);
        let op = if cleanup {
            &mut s.magnetic.cleanup
        } else {
            &mut s.operation
        };
        if let Some(op) = op {
            change(op);
        }
    }
    async fn execute(&mut self, command: DeviceCommand, cleanup: bool) -> Result<()> {
        let previous = lock(&self.shared).magnetic.phase;
        if !cleanup {
            let calsw = self.read_register(1, None).await?[0];
            if (matches!(command, DeviceCommand::MagStart | DeviceCommand::Save) && calsw != 0)
                || (matches!(command, DeviceCommand::MagStop) && ![0, 7].contains(&calsw))
            {
                return Err(Error::Invalid(format!(
                    "magnetic command not allowed with CALSW={calsw}"
                )));
            }
            if matches!(command, DeviceCommand::MagStart) {
                // A new read proves the magnetic path works before any calibration write.
                self.read_register(0x3a, None).await?;
            }
        }
        lock(&self.shared).magnetic.phase = match command {
            DeviceCommand::MagStart => MagneticPhase::Starting,
            DeviceCommand::MagStop => MagneticPhase::Stopping,
            _ => MagneticPhase::Saving,
        };
        self.write(&protocol::UNLOCK).await?;
        self.delay(Duration::from_millis(200)).await?;
        {
            let mut s = lock(&self.shared);
            s.descriptor.device.valid = false;
            s.descriptor.metadata_revision += 1;
            if !matches!(command, DeviceCommand::Save) {
                s.descriptor.reference_epoch += 1;
                s.descriptor.reference_reason = if cleanup {
                    "magnetic_cleanup"
                } else {
                    "magnetic_control"
                }
                .into();
            }
            if matches!(command, DeviceCommand::MagStart) {
                s.magnetic.owned = true;
            }
        }
        self.operation(cleanup, |op| {
            op.write_attempted = true;
            op.reference_may_have_changed = !matches!(command, DeviceCommand::Save);
            if matches!(command, DeviceCommand::Save) {
                op.persistence = "unverified".into();
            }
        });
        let (address, value) = protocol::command_register(&command)?;
        self.write(&protocol::write_register(address, value))
            .await?;
        self.operation(cleanup, |op| op.command_sent = true);
        if matches!(command, DeviceCommand::Save) {
            lock(&self.shared).magnetic.phase = previous;
            self.operation(cleanup, |op| {
                op.outcome = OperationOutcome::Unverified;
                op.message = Some("SAVE sent; power-cycle persistence unverified".into());
            });
            return Ok(());
        }
        self.delay(Duration::from_millis(100)).await?;
        self.read_register(1, Some(value)).await?;
        {
            let mut s = lock(&self.shared);
            if matches!(command, DeviceCommand::MagStart) {
                s.magnetic.window = Window::default();
                s.magnetic.phase = MagneticPhase::Calibrating;
            } else {
                s.magnetic.owned = false;
                s.magnetic.window.end.get_or_insert(Instant::now());
                s.magnetic.phase = MagneticPhase::Calibrated;
            }
        }
        self.operation(cleanup, |op| {
            op.register_verified = true;
            op.completion_observed = matches!(command, DeviceCommand::MagStop);
            op.outcome = OperationOutcome::Succeeded;
            op.message = Some("CALSW verified; accuracy unverified; no SAVE sent".into());
        });
        Ok(())
    }
    async fn monitor(&mut self, commands: &mut mpsc::Receiver<DeviceCommand>) -> Result<()> {
        match self.read_register(0x72, None).await {
            Ok(values) => {
                let mut s = lock(&self.shared);
                s.magnetic.sensor_type = Some(values[0]);
                if scale(values[0]).is_none() {
                    s.magnetic.type_error = Some(format!(
                        "unknown magnetic sensor type {}; units unavailable",
                        values[0]
                    ));
                }
            }
            Err(Error::Cancelled) => return Err(Error::Cancelled),
            Err(e) => lock(&self.shared).magnetic.type_error = Some(e.to_string()),
        }
        let calsw = self.read_register(1, None).await?[0];
        lock(&self.shared).magnetic.phase = if calsw == 7 {
            MagneticPhase::ExternalCalibration
        } else {
            MagneticPhase::Monitoring
        };
        self.read_register(0x3a, None).await?;
        let mut next_mag = Instant::now() + POLL_INTERVAL;
        let mut next_control = Instant::now() + Duration::from_secs(1);
        let mut cancel = self.cancel.clone();
        loop {
            if *cancel.borrow() {
                return Err(Error::Cancelled);
            }
            tokio::select! {
                biased;
                _ = cancel.changed() => return Err(Error::Cancelled),
                command = commands.recv() => {
                    let command = command.ok_or(Error::Cancelled)?;
                    let result = self.execute(command, false).await;
                    if let Err(e) = result {
                        self.operation(false, |op| {
                            op.outcome = if matches!(e, Error::Cancelled) { OperationOutcome::Cancelled } else { OperationOutcome::Failed };
                            op.message = Some(e.to_string());
                        });
                        if !matches!(e, Error::Invalid(_)) { return Err(e); }
                    }
                }
                _ = tokio::time::sleep_until(next_mag.min(next_control).into()) => {
                    if next_control <= next_mag {
                        next_control = Instant::now() + Duration::from_secs(1);
                        self.read_register(1, None).await?;
                    } else {
                        next_mag = Instant::now() + POLL_INTERVAL;
                        self.read_register(0x3a, None).await?;
                    }
                }
                frames = self.receive() => { frames?; }
            }
        }
    }
}

pub(super) async fn run(
    source: Source,
    shared: SharedRef,
    mut cancel: watch::Receiver<bool>,
    mut commands: mpsc::Receiver<DeviceCommand>,
) -> Result<()> {
    let mut connection = match Connection::open(&source, &mut cancel).await {
        Ok(c) => c,
        Err(e) => {
            lock(&shared)
                .magnetic
                .finish((!matches!(e, Error::Cancelled)).then(|| e.to_string()));
            return Err(e);
        }
    };
    let result = run_connection(&mut connection, shared.clone(), cancel, &mut commands).await;
    // Cleanup plus close must fit the Controller's five-second stop budget.
    let _ = tokio::time::timeout(Duration::from_millis(500), connection.close()).await;
    result
}

async fn run_connection(
    connection: &mut Connection,
    shared: SharedRef,
    cancel: watch::Receiver<bool>,
    commands: &mut mpsc::Receiver<DeviceCommand>,
) -> Result<()> {
    let (reader, writer) = connection.split();
    let mut io = Io {
        reader,
        writer,
        parser: Parser::default(),
        shared: shared.clone(),
        cancel,
        start: Instant::now(),
    };
    lock(&shared).status.state = ConnectionState::Magnetic;
    let mut result = io.monitor(commands).await;
    lock(&shared).magnetic.running = false;
    if lock(&shared).magnetic.owned {
        let mut op = OperationStatus::new("mag_stop_cleanup".into());
        op.source_id = Some(lock(&shared).descriptor.source_id.clone());
        lock(&shared).magnetic.cleanup = Some(op);
        let (_cleanup_tx, cleanup_rx) = watch::channel(false);
        io.cancel = cleanup_rx;
        let cleanup = tokio::time::timeout(
            Duration::from_secs(4),
            io.execute(DeviceCommand::MagStop, true),
        )
        .await
        .unwrap_or_else(|_| {
            Err(Error::Timeout(
                "magnetic cleanup exceeded four seconds".into(),
            ))
        });
        if let Err(e) = cleanup {
            let message =
                format!("{e}; device may still be calibrating; reconnect and explicitly mag_stop");
            io.operation(true, |op| {
                op.outcome = OperationOutcome::Failed;
                op.message = Some(message.clone());
            });
            result = Err(Error::Io(message));
        }
    }
    let error = result
        .as_ref()
        .err()
        .filter(|e| !matches!(e, Error::Cancelled))
        .map(ToString::to_string);
    lock(&shared).magnetic.finish(error);
    result
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn units_are_signed_and_unknown_types_do_not_invent_microteslas() {
        for (kind, expected) in [
            (2, 0.15),
            (3, 0.013),
            (4, 0.058),
            (5, 0.098),
            (6, 1.0 / 120.0),
            (7, 0.02),
        ] {
            let now = Instant::now();
            let mut s = State::new(1, 2);
            s.sensor_type = Some(kind);
            s.observe([-120, 240, i16::MIN], now, now);
            let sample = &s.records[0].1;
            assert_eq!(
                sample.field_ut,
                Some([
                    -120.0 * expected,
                    240.0 * expected,
                    f64::from(i16::MIN) * expected
                ])
            );
        }
        for kind in [None, Some(0), Some(1), Some(65535)] {
            let now = Instant::now();
            let mut s = State::new(1, 2);
            s.sensor_type = kind;
            s.observe([0, -1, 32767], now, now);
            assert!(s.records[0].1.field_ut.is_none());
        }
    }

    #[test]
    fn bounded_queries_windows_and_age_are_independent_of_poses() {
        let now = Instant::now();
        let mut s = State::new(1, 2);
        s.phase = MagneticPhase::Monitoring;
        for n in 0..300 {
            s.observe(
                [n as i16, -(n as i16), 7],
                now + Duration::from_millis(n * 200),
                now,
            );
        }
        let cursor = MagneticCursor {
            instance_id: 1,
            session_id: 2,
            sequence: 1,
        };
        let query = now + Duration::from_secs(60);
        let batch = s.since(Some(cursor), "test".into(), None, query);
        assert_eq!(batch.history_overrun, 43);
        assert_eq!(batch.samples.len(), 256);
        assert_eq!(batch.statistics.sample_count, 300);
        assert_eq!(batch.statistics.span_counts, Some([299, 299, 0]));
        assert!((batch.statistics.actual_rate_hz - 5.0).abs() < 1e-10);
        assert!(batch.active && batch.latest.as_ref().unwrap().fresh);
        assert!(!batch.samples[0].fresh);
        assert!(
            s.since(batch.cursor, "test".into(), None, query)
                .samples
                .is_empty()
        );
        let old_window = batch.statistics.window_id;
        s.window = Window::default();
        s.observe([i16::MIN, 0, 0], query, now);
        s.observe([i16::MAX, 0, 0], query, now);
        let reset_window = s.since(batch.cursor, "test".into(), None, query);
        assert_ne!(reset_window.statistics.window_id, old_window);
        assert_eq!(reset_window.samples[0].cursor.sequence, 301);
        assert_eq!(reset_window.statistics.span_counts, Some([65535, 0, 0]));
        s.running = false;
        s.window.end = Some(query);
        let stopped = s.since(None, "test".into(), None, query + Duration::from_secs(3));
        assert!(!stopped.active && stopped.samples.iter().all(|s| !s.fresh));
        assert_eq!(stopped.latest.unwrap().age_ns, 3_000_000_000);
        assert_eq!(stopped.statistics.elapsed_ns, 0);
        let replacement = State::new(3, 4).since(batch.cursor, "test".into(), None, query);
        assert!(replacement.reset && replacement.samples.is_empty());
    }

    #[test]
    fn cursor_json_is_strict_and_roundtrips() {
        let cursor = MagneticCursor {
            instance_id: 1,
            session_id: 2,
            sequence: 3,
        };
        let json = serde_json::to_string(&cursor).unwrap();
        assert_eq!(
            serde_json::from_str::<MagneticCursor>(&json).unwrap(),
            cursor
        );
        for sequence in ["3", "\"-1\"", "\"+1\"", "\"9223372036854775808\"", "null"] {
            let json =
                format!("{{\"instance_id\":\"1\",\"session_id\":\"2\",\"sequence\":{sequence}}}");
            assert!(
                serde_json::from_str::<MagneticCursor>(&json).is_err(),
                "{json}"
            );
        }
    }

    #[cfg(unix)]
    mod wire {
        use super::*;
        use crate::controller::Shared;
        use std::sync::{Arc, Mutex};
        use tokio::io::{AsyncReadExt, AsyncWriteExt};
        use tokio_serial::SerialStream;

        #[derive(Default)]
        struct Device {
            calsw: u16,
            commands: Vec<[u8; 5]>,
            ignore_stop: bool,
            hide_start: bool,
            silence_mag: bool,
            silence_type: bool,
        }
        struct Fixture {
            shared: SharedRef,
            device: Arc<Mutex<Device>>,
            tx: mpsc::Sender<DeviceCommand>,
            cancel: watch::Sender<bool>,
            worker: tokio::task::JoinHandle<Result<()>>,
            server: tokio::task::JoinHandle<()>,
        }
        impl Fixture {
            async fn new(calsw: u16) -> Self {
                Self::with_device(Device {
                    calsw,
                    ..Device::default()
                })
                .await
            }
            async fn with_device(initial: Device) -> Self {
                let (mut serial, host) = SerialStream::pair().unwrap();
                let device = Arc::new(Mutex::new(initial));
                let observed = device.clone();
                let server = tokio::spawn(async move {
                    let mut cmd = [0; 5];
                    while serial.read_exact(&mut cmd).await.is_ok() {
                        let response = {
                            let mut d = observed.lock().unwrap();
                            d.commands.push(cmd);
                            if cmd[2] == 1 {
                                let value = u16::from_le_bytes([cmd[3], cmd[4]]);
                                if value != 0 || !d.ignore_stop {
                                    d.calsw = value;
                                }
                            }
                            if cmd[2] != 0x27
                                || (cmd[3] == 0x3a && d.silence_mag)
                                || (cmd[3] == 0x72 && d.silence_type)
                            {
                                None
                            } else {
                                let mut words = [0i16; 8];
                                match cmd[3] {
                                    0x72 => words[0] = 6,
                                    1 => {
                                        words[0] = if d.hide_start && d.calsw == 7 {
                                            0
                                        } else {
                                            d.calsw as i16
                                        }
                                    }
                                    0x3a => words[..3].copy_from_slice(&[120, -240, 60]),
                                    _ => {}
                                }
                                let mut bytes = vec![0x55, 0x61];
                                bytes.extend([0; 18]); // ignored pose
                                bytes.extend([0x55, 0x71, cmd[3], cmd[4]]);
                                for word in words {
                                    bytes.extend(word.to_le_bytes());
                                }
                                Some(bytes)
                            }
                        };
                        if let Some(bytes) = response {
                            // Exercise a split reply and mixed frames under one parser.
                            if serial.write_all(&bytes[..23]).await.is_err() {
                                break;
                            }
                            if serial.write_all(&bytes[23..]).await.is_err() {
                                break;
                            }
                        }
                    }
                });
                let shared = Arc::new(Mutex::new(Shared::default()));
                lock(&shared).magnetic = State::new(1, 2);
                let (tx, mut rx) = mpsc::channel(1);
                let (cancel, crx) = watch::channel(false);
                let state = shared.clone();
                let worker = tokio::spawn(async move {
                    let mut connection = Connection::Usb(host);
                    run_connection(&mut connection, state, crx, &mut rx).await
                });
                let f = Self {
                    shared,
                    device,
                    tx,
                    cancel,
                    worker,
                    server,
                };
                f.wait(|s| !s.magnetic.records.is_empty()).await;
                f
            }
            async fn wait(&self, predicate: impl Fn(&Shared) -> bool) {
                tokio::time::timeout(Duration::from_secs(6), async {
                    loop {
                        if predicate(&lock(&self.shared)) {
                            break;
                        }
                        tokio::time::sleep(Duration::from_millis(5)).await;
                    }
                })
                .await
                .unwrap();
            }
            async fn command(&self, command: DeviceCommand) {
                lock(&self.shared).operation = Some(OperationStatus::new(format!("{command:?}")));
                self.tx.send(command).await.unwrap();
                self.wait(|s| s.operation.as_ref().unwrap().outcome != OperationOutcome::Running)
                    .await;
            }
            async fn finish(self) -> (Result<()>, SharedRef, Vec<[u8; 5]>) {
                self.cancel.send(true).ok();
                let result = tokio::time::timeout(Duration::from_secs(5), self.worker)
                    .await
                    .unwrap()
                    .unwrap();
                self.server.abort();
                let _ = self.server.await;
                let commands = self.device.lock().unwrap().commands.clone();
                (result, self.shared, commands)
            }
        }
        fn writes(commands: &[[u8; 5]], address: u8, value: u16) -> usize {
            commands
                .iter()
                .filter(|c| **c == protocol::write_register(address, value))
                .count()
        }

        #[tokio::test]
        async fn monitor_and_external_calibration_are_read_only() {
            for calsw in [0, 7] {
                let f = Fixture::new(calsw).await;
                f.wait(|s| s.magnetic.sequence >= 2).await;
                assert!(lock(&f.shared).pose.is_none());
                assert_eq!(lock(&f.shared).status.pose_count, 0);
                let (result, s, commands) = f.finish().await;
                assert!(matches!(result, Err(Error::Cancelled)));
                assert!(commands.iter().all(|c| c[2] == 0x27));
                let s = lock(&s);
                assert!(!s.magnetic.running);
                assert_eq!(
                    s.magnetic.records.back().unwrap().1.field_ut,
                    Some([1.0, -2.0, 0.5])
                );
                assert_eq!(s.magnetic.calsw, Some(calsw));
            }
        }

        #[tokio::test]
        async fn full_control_flow_freezes_window_and_only_saves_explicitly() {
            let f = Fixture::new(0).await;
            f.command(DeviceCommand::MagStart).await;
            assert_eq!(lock(&f.shared).magnetic.phase, MagneticPhase::Calibrating);
            f.wait(|s| s.magnetic.window.count >= 2).await;
            let reference = lock(&f.shared).descriptor.reference_epoch;
            f.command(DeviceCommand::MagStop).await;
            assert!(lock(&f.shared).descriptor.reference_epoch > reference);
            let count = lock(&f.shared).magnetic.window.count;
            let sequence = lock(&f.shared).magnetic.sequence;
            f.wait(|s| s.magnetic.sequence > sequence).await;
            assert_eq!(lock(&f.shared).magnetic.window.count, count);
            f.command(DeviceCommand::Save).await;
            assert_eq!(
                lock(&f.shared).operation.as_ref().unwrap().outcome,
                OperationOutcome::Unverified
            );
            let (_, s, commands) = f.finish().await;
            assert!(lock(&s).magnetic.cleanup.is_none());
            assert_eq!(writes(&commands, 1, 7), 1);
            assert_eq!(writes(&commands, 1, 0), 1);
            assert_eq!(writes(&commands, 0, 0), 1);
        }

        #[tokio::test]
        async fn cancel_owned_calibration_stops_but_never_saves() {
            let f = Fixture::new(0).await;
            f.command(DeviceCommand::MagStart).await;
            let (result, s, commands) = f.finish().await;
            assert!(matches!(result, Err(Error::Cancelled)));
            let s = lock(&s);
            assert_eq!(
                s.magnetic.cleanup.as_ref().unwrap().outcome,
                OperationOutcome::Succeeded
            );
            assert!(!s.magnetic.owned);
            assert_eq!(writes(&commands, 1, 7), 1);
            assert_eq!(writes(&commands, 1, 0), 1);
            assert_eq!(writes(&commands, 0, 0), 0);
        }

        #[tokio::test]
        async fn ambiguous_start_cleans_up_without_repeating_start() {
            let f = Fixture::new(0).await;
            f.device.lock().unwrap().hide_start = true;
            f.command(DeviceCommand::MagStart).await;
            let (result, s, commands) = f.finish().await;
            assert!(result.is_err());
            let s = lock(&s);
            assert_eq!(
                s.operation.as_ref().unwrap().outcome,
                OperationOutcome::Failed
            );
            assert!(s.operation.as_ref().unwrap().write_attempted);
            assert_eq!(
                s.magnetic.cleanup.as_ref().unwrap().outcome,
                OperationOutcome::Succeeded
            );
            assert_eq!(writes(&commands, 1, 7), 1);
            assert_eq!(writes(&commands, 1, 0), 1);
            assert_eq!(writes(&commands, 0, 0), 0);
        }

        #[tokio::test]
        async fn cleanup_failure_is_bounded_and_preserves_uncertainty() {
            let f = Fixture::new(0).await;
            f.command(DeviceCommand::MagStart).await;
            f.device.lock().unwrap().ignore_stop = true;
            let (result, s, commands) = f.finish().await;
            assert!(result.is_err());
            let s = lock(&s);
            assert_eq!(s.magnetic.phase, MagneticPhase::Failed);
            assert!(
                s.magnetic.owned
                    && s.magnetic
                        .last_error
                        .as_ref()
                        .unwrap()
                        .contains("may still be calibrating")
            );
            assert_eq!(
                s.magnetic.cleanup.as_ref().unwrap().outcome,
                OperationOutcome::Failed
            );
            assert_eq!(writes(&commands, 1, 0), 1);
            assert_eq!(writes(&commands, 0, 0), 0);
        }

        #[tokio::test]
        async fn preconditions_and_cancel_before_start_write_do_not_calibrate() {
            let f = Fixture::new(7).await;
            f.command(DeviceCommand::MagStart).await;
            f.command(DeviceCommand::Save).await;
            let (_, _, commands) = f.finish().await;
            assert!(commands.iter().all(|c| c[2] == 0x27));
            let f = Fixture::new(0).await;
            lock(&f.shared).operation = Some(OperationStatus::new("mag_start".into()));
            f.tx.send(DeviceCommand::MagStart).await.unwrap();
            tokio::time::timeout(Duration::from_secs(2), async {
                while !f
                    .device
                    .lock()
                    .unwrap()
                    .commands
                    .contains(&protocol::UNLOCK)
                {
                    tokio::time::sleep(Duration::from_millis(5)).await;
                }
            })
            .await
            .unwrap();
            let (_, s, commands) = f.finish().await;
            assert!(lock(&s).magnetic.cleanup.is_none());
            assert_eq!(writes(&commands, 1, 7), 0);
            assert_eq!(writes(&commands, 1, 0), 0);
        }

        #[tokio::test]
        async fn magnetic_timeout_does_not_retry_or_reconnect() {
            let f = Fixture::new(0).await;
            f.device.lock().unwrap().silence_mag = true;
            f.wait(|s| s.magnetic.phase == MagneticPhase::Failed).await;
            let (result, _, commands) = f.finish().await;
            assert!(matches!(result, Err(Error::Timeout(_))));
            assert_eq!(
                commands
                    .iter()
                    .filter(|c| c[2] == 0x27 && c[3] == 0x3a)
                    .count(),
                2
            );
        }

        #[tokio::test]
        async fn unavailable_type_still_delivers_counts_without_writes() {
            let f = Fixture::with_device(Device {
                silence_type: true,
                ..Device::default()
            })
            .await;
            {
                let s = lock(&f.shared);
                assert!(s.magnetic.type_error.is_some());
                assert!(s.magnetic.sensor_type.is_none());
                assert!(s.magnetic.records.back().unwrap().1.field_ut.is_none());
            }
            let (_, _, commands) = f.finish().await;
            assert!(commands.iter().all(|c| c[2] == 0x27));
        }

        #[tokio::test]
        async fn disconnect_keeps_owned_calibration_uncertain_without_restart_or_save() {
            let f = Fixture::new(0).await;
            f.command(DeviceCommand::MagStart).await;
            f.server.abort();
            f.wait(|s| s.magnetic.phase == MagneticPhase::Failed).await;
            let (result, shared, commands) = f.finish().await;
            assert!(result.is_err());
            let s = lock(&shared);
            assert!(s.magnetic.owned && !s.magnetic.running);
            assert_eq!(
                s.magnetic.cleanup.as_ref().unwrap().outcome,
                OperationOutcome::Failed
            );
            assert_eq!(writes(&commands, 1, 7), 1);
            assert_eq!(writes(&commands, 0, 0), 0);
        }
    }
}
