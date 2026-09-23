use crate::battery::{self, Battery};
use crate::device::{self, Progress};
use crate::model::*;
use crate::motion::{History, Signals};
use crate::osc::{Sender, Telemetry};
use crate::pose;
use crate::protocol::{self, Frame, Parser};
use crate::sample_clock::SampleClock;
use crate::transport::{self, Connection};
use crate::{MotionBatch, MotionCursor, MotionSample, OrientationSource};
use futures_util::{FutureExt, StreamExt};
use std::future::Future;
use std::panic::AssertUnwindSafe;
use std::sync::{Arc, Mutex, MutexGuard, mpsc};
use std::time::{Duration, Instant};
use tokio::runtime::Runtime;
use tokio::sync::watch;

#[path = "magnetic.rs"]
pub(crate) mod magnetic;
use magnetic::{MagneticBatch, MagneticCursor};

pub const STALE_AFTER: Duration = Duration::from_millis(500);

#[derive(Default)]
struct Shared {
    status: StatusSnapshot,
    descriptor: SourceDescriptor,
    operation: Option<OperationStatus>,
    pose: Option<PoseSnapshot>,
    devices: Vec<DeviceInfo>,
    last_pose: Option<Instant>,
    first_pose: Option<Instant>,
    motion: History,
    battery: Battery,
    magnetic: magnetic::State,
}
type SharedRef = Arc<Mutex<Shared>>;

fn lock(shared: &SharedRef) -> MutexGuard<'_, Shared> {
    shared.lock().unwrap_or_else(|p| p.into_inner())
}

fn fresh(s: &Shared, now: Instant) -> bool {
    matches!(
        s.status.state,
        ConnectionState::Active | ConnectionState::Stale
    ) && s
        .last_pose
        .is_some_and(|t| now.duration_since(t) < STALE_AFTER)
}

struct Task {
    cancel: watch::Sender<bool>,
    handle: tokio::task::JoinHandle<()>,
    done: mpsc::Receiver<()>,
}
struct CompleteOnDrop(mpsc::Sender<()>);
impl Drop for CompleteOnDrop {
    fn drop(&mut self) {
        let _ = self.0.send(());
    }
}

/// Synchronous control facade shared by CLI and C callers. Not an audio callback API.
pub struct Controller {
    runtime: Option<Runtime>,
    shared: SharedRef,
    config: Config,
    task: Option<Task>,
    magnetic_commands: Option<tokio::sync::mpsc::Sender<DeviceCommand>>,
}

impl Controller {
    pub fn new() -> Result<Self> {
        let runtime = tokio::runtime::Builder::new_multi_thread()
            .worker_threads(2)
            .thread_name("posebridge")
            .enable_all()
            .build()?;
        Ok(Self {
            runtime: Some(runtime),
            shared: Arc::new(Mutex::new(Shared::default())),
            config: Config::default(),
            task: None,
            magnetic_commands: None,
        })
    }

    fn ensure_idle(&mut self) -> Result<()> {
        if self.task.as_ref().is_some_and(|t| !t.handle.is_finished()) {
            return Err(Error::Busy);
        }
        if self.task.is_some() {
            self.stop()?;
        }
        Ok(())
    }

    pub fn set_config(&mut self, config: Config) -> Result<()> {
        config.validate()?;
        self.ensure_idle()?;
        let mut s = lock(&self.shared);
        let changed = serde_json::to_value(&self.config).ok() != serde_json::to_value(&config).ok();
        if changed {
            let same_device = serde_json::to_value(&self.config.source).ok()
                == serde_json::to_value(&config.source).ok();
            let old = s.descriptor.clone();
            let reference_changed = !same_device
                || self.config.mounting != config.mounting
                || self.config.pose_input != config.pose_input;
            s.descriptor = SourceDescriptor::new(&config);
            s.descriptor.metadata_revision = old.metadata_revision + 1;
            s.descriptor.reference_epoch = old.reference_epoch + u64::from(reference_changed);
            s.descriptor.reference_reason = if reference_changed {
                "application_configuration_changed".into()
            } else {
                old.reference_reason
            };
            if same_device {
                s.descriptor.device = old.device;
                s.descriptor.device_name = old.device_name;
            }
            s.motion = History::default();
            s.magnetic = magnetic::State::default();
            s.pose = None;
            s.last_pose = None;
            s.status = StatusSnapshot::default();
            s.battery = Battery::default();
        }
        self.config = config;
        Ok(())
    }

    pub fn config(&self) -> &Config {
        &self.config
    }

    pub fn start(&mut self) -> Result<()> {
        self.config.validate_acquisition()?;
        self.ensure_idle()?;
        let config = self.config.clone();
        {
            let mut s = lock(&self.shared);
            s.status = StatusSnapshot::default();
            s.battery = Battery::default();
            s.status.state = ConnectionState::Connecting;
            s.descriptor.instance_id = new_id();
            s.descriptor.session_id = 0;
            s.descriptor.metadata_revision += 1;
            s.motion = History::default();
            s.magnetic = magnetic::State::default();
            s.pose = None;
            s.last_pose = None;
            s.first_pose = None;
        }
        self.launch(config.osc.clone(), move |shared, cancel| {
            run(config, shared, cancel)
        });
        Ok(())
    }

    pub fn scan_start(&mut self, kind: TransportKind, seconds: u32) -> Result<()> {
        if !(1..=60).contains(&seconds) {
            return Err(Error::Invalid("scan duration must be 1..60 seconds".into()));
        }
        self.ensure_idle()?;
        {
            let mut s = lock(&self.shared);
            s.devices.clear();
            s.magnetic = magnetic::State::default();
            s.battery = Battery::default();
            s.status.state = ConnectionState::Scanning;
        }
        self.launch(None, move |shared, mut cancel| async move {
            let devices =
                transport::scan(kind, Duration::from_secs(seconds as u64), &mut cancel).await?;
            lock(&shared).devices = devices;
            Ok(())
        });
        Ok(())
    }

    pub fn inspect_start(&mut self) -> Result<()> {
        self.config.validate()?;
        self.ensure_idle()?;
        if matches!(self.config.source, Source::Simulate { .. }) {
            return Err(Error::Invalid("simulator has no device registers".into()));
        }
        let source = self.config.source.clone();
        {
            let mut s = lock(&self.shared);
            s.status.state = ConnectionState::Inspecting;
            s.magnetic = magnetic::State::default();
            s.status.last_error = None;
            s.status.configuration_report = None;
            s.battery = Battery::default();
            s.descriptor.device.valid = false;
            s.descriptor.metadata_revision += 1;
            let mut operation = OperationStatus::new("inspect".into());
            operation.source_id = Some(s.descriptor.source_id.clone());
            s.operation = Some(operation);
        }
        self.launch(None, move |shared, mut cancel| async move {
            let mut connection = Connection::open(&source, &mut cancel).await?;
            let name = connection.device_name(&source).await;
            let result = async {
                let observation = device::inspect(&mut connection, &mut cancel).await?;
                let mut battery = Battery::default();
                match device::read_registers(&mut connection, battery::REGISTER, None, &mut cancel).await {
                    Ok(values) => battery.observe(values[0], Instant::now()),
                    Err(Error::Cancelled) => return Err(Error::Cancelled),
                    Err(error) => battery.fail(error.to_string()),
                }
                Ok((observation, battery))
            }.await;
            connection.close().await;
            let (observation, battery) = result?;
            let mut s = lock(&shared);
            s.battery = battery;
            s.descriptor.device = observation;
            s.descriptor.device_name = name;
            s.descriptor.metadata_revision += 1;
            if let Some(op) = &mut s.operation {
                op.register_verified = true; op.outcome = OperationOutcome::Succeeded;
                op.message = Some("read-only inspection complete; capabilities and calibration quality are not inferred from register access".into());
            }
            Ok(())
        });
        Ok(())
    }

    pub fn configure_device(&mut self, command: DeviceCommand) -> Result<()> {
        self.config.validate()?;
        protocol::command_register(&command)?;
        if let Some(tx) = &self.magnetic_commands
            && self.task.as_ref().is_some_and(|t| !t.handle.is_finished())
        {
            let mut s = lock(&self.shared);
            if !matches!(
                command,
                DeviceCommand::MagStart | DeviceCommand::MagStop | DeviceCommand::Save
            ) || !s.magnetic.ready()
                || s.operation
                    .as_ref()
                    .is_some_and(|op| op.outcome == OperationOutcome::Running)
            {
                return Err(Error::Busy);
            }
            let action = match command {
                DeviceCommand::MagStart => "mag_start",
                DeviceCommand::MagStop => "mag_stop",
                _ => "save",
            };
            let mut operation = OperationStatus::new(action.into());
            operation.source_id = Some(s.descriptor.source_id.clone());
            // Keep the shared lock until the operation is published: the worker
            // cannot complete the queued command before its status exists.
            tx.try_send(command).map_err(|_| Error::Busy)?;
            s.operation = Some(operation);
            return Ok(());
        }
        self.ensure_idle()?;
        if matches!(self.config.source, Source::Simulate { .. }) {
            return Err(Error::Invalid("simulator has no device registers".into()));
        }
        let source = self.config.source.clone();
        let action =
            serde_json::to_value(&command).map_err(|e| Error::Internal(e.to_string()))?["action"]
                .as_str()
                .unwrap_or("unknown")
                .to_string();
        {
            let mut s = lock(&self.shared);
            s.status.state = ConnectionState::Configuring;
            s.magnetic = magnetic::State::default();
            s.battery = Battery::default();
            s.status.last_error = None;
            s.status.configuration_report = None;
            let mut operation = OperationStatus::new(action);
            operation.source_id = Some(s.descriptor.source_id.clone());
            s.operation = Some(operation);
        }
        self.launch(None, move |shared, mut cancel| async move {
            let mut connection = Connection::open(&source, &mut cancel).await?;
            let result = control_connection(&shared, &mut connection, &command, &mut cancel).await;
            connection.close().await;
            result
        });
        Ok(())
    }

    /// Open an exclusive, read-only magnetic monitor; use explicit device
    /// commands to begin/end calibration or save. No mounting is required.
    pub fn magnetic_start(&mut self) -> Result<()> {
        self.config.validate()?;
        self.ensure_idle()?;
        if matches!(self.config.source, Source::Simulate { .. }) {
            return Err(Error::Invalid("simulator has no magnetic registers".into()));
        }
        let source = self.config.source.clone();
        {
            let mut s = lock(&self.shared);
            s.status = StatusSnapshot::default();
            s.status.state = ConnectionState::Connecting;
            s.descriptor.instance_id = new_id();
            s.descriptor.session_id = new_id();
            s.status.session_id = s.descriptor.session_id;
            s.descriptor.metadata_revision += 1;
            s.descriptor.device.valid = false;
            s.magnetic = magnetic::State::new(s.descriptor.instance_id, s.descriptor.session_id);
            s.motion = History::default();
            s.battery = Battery::default();
            s.pose = None;
            s.last_pose = None;
            s.first_pose = None;
            s.operation = None;
        }
        let (tx, rx) = tokio::sync::mpsc::channel(1);
        self.magnetic_commands = Some(tx);
        self.launch(None, move |shared, cancel| {
            magnetic::run(source, shared, cancel, rx)
        });
        Ok(())
    }

    /// Non-consuming incremental magnetic query, including status and operation
    /// outcome under the same lock. Never performs device I/O.
    pub fn magnetic_since(&self, cursor: Option<MagneticCursor>) -> MagneticBatch {
        let s = lock(&self.shared);
        s.magnetic.since(
            cursor,
            s.descriptor.source_id.clone(),
            (s.magnetic.phase != magnetic::MagneticPhase::Idle)
                .then(|| s.operation.clone())
                .flatten(),
            Instant::now(),
        )
    }

    fn launch<F, Fut>(&mut self, osc: Option<OscConfig>, work: F)
    where
        F: FnOnce(SharedRef, watch::Receiver<bool>) -> Fut + Send + 'static,
        Fut: Future<Output = Result<()>> + Send + 'static,
    {
        let (cancel, rx) = watch::channel(false);
        let (done_tx, done) = mpsc::channel();
        let shared = self.shared.clone();
        let complete = CompleteOnDrop(done_tx);
        let handle = self
            .runtime
            .as_ref()
            .expect("live controller runtime")
            .spawn(async move {
                let _complete = complete;
                let future = AssertUnwindSafe(work(shared.clone(), rx)).catch_unwind();
                tokio::pin!(future);
                let mut telemetry = osc.as_ref().and_then(|config| match Telemetry::new(config) {
                    Ok(value) => Some(value),
                    Err(e) => { lock(&shared).status.last_error = Some(e.to_string()); None }
                });
                let mut tick = tokio::time::interval(Duration::from_millis(20));
                tick.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
                let result = loop {
                    tokio::select! {
                        result = &mut future => break result,
                        _ = tick.tick(), if telemetry.is_some() => send_telemetry(&shared, &mut telemetry, false),
                    }
                };
                {
                    let mut s = lock(&shared);
                    match result {
                        Ok(Ok(())) => s.status.state = ConnectionState::Complete,
                        Ok(Err(Error::Cancelled)) => {
                            s.status.state = ConnectionState::Stopped;
                            if let Some(op) = &mut s.operation && op.outcome == OperationOutcome::Running {
                                op.outcome = OperationOutcome::Cancelled;
                            }
                        },
                        Ok(Err(e)) => {
                            s.status.state = ConnectionState::Failed;
                            s.status.last_error = Some(e.to_string());
                            if let Some(op) = &mut s.operation && op.outcome == OperationOutcome::Running {
                                op.outcome = OperationOutcome::Failed; op.message = Some(e.to_string());
                            }
                        },
                        Err(_) => {
                            s.status.state = ConnectionState::Failed;
                            s.status.last_error = Some("internal worker panic".into());
                            if !matches!(s.magnetic.phase, magnetic::MagneticPhase::Idle | magnetic::MagneticPhase::Closed | magnetic::MagneticPhase::Failed) {
                                s.magnetic.finish(Some("internal worker panic; device calibration state unknown".into()));
                            }
                            if let Some(op) = &mut s.operation && op.outcome == OperationOutcome::Running { op.outcome = OperationOutcome::Failed; }
                        },
                    }
                }
                send_telemetry(&shared, &mut telemetry, true);
            });
        self.task = Some(Task {
            cancel,
            handle,
            done,
        });
    }

    pub fn stop(&mut self) -> Result<()> {
        let magnetic = self.magnetic_commands.take().is_some();
        if let Some(task) = self.task.take() {
            let _ = task.cancel.send(true);
            if task.done.recv_timeout(Duration::from_secs(5)).is_err() {
                task.handle.abort();
                let _ = task.done.recv_timeout(Duration::from_secs(1));
                let mut s = lock(&self.shared);
                s.status.state = ConnectionState::Failed;
                s.status.last_error = Some("stop timed out; worker aborted".into());
                if magnetic {
                    s.magnetic.finish(Some(
                        "stop timed out; device calibration state unknown".into(),
                    ));
                }
                return Err(Error::Timeout("stop; worker aborted".into()));
            }
        }
        if magnetic {
            let s = lock(&self.shared);
            if let Some(error) = &s.magnetic.last_error {
                return Err(Error::Io(error.clone()));
            }
        }
        lock(&self.shared).status.state = ConnectionState::Stopped;
        Ok(())
    }

    pub fn snapshot(&self) -> Snapshot {
        snapshot(&self.shared)
    }
    pub fn status(&self) -> StatusSnapshot {
        self.snapshot().status
    }
    /// Read all retained motion records after a cursor. A delivery is never split
    /// by publication; no device/JSON/audio work occurs while copying the batch.
    pub fn motion_since(&self, cursor: Option<MotionCursor>) -> MotionBatch {
        let s = lock(&self.shared);
        let now = Instant::now();
        let mut batch = s.motion.since(cursor, now, fresh(&s, now));
        batch.source_id = s.descriptor.source_id.clone();
        if cursor.is_some_and(|c| {
            c.instance_id != s.descriptor.instance_id || c.session_id != s.status.session_id
        }) {
            batch.reset = true;
        }
        batch
    }

    pub fn latest_pose(&self) -> Option<PoseSnapshot> {
        self.snapshot().pose
    }

    pub fn devices(&self) -> Vec<DeviceInfo> {
        lock(&self.shared).devices.clone()
    }
}

impl Drop for Controller {
    fn drop(&mut self) {
        let _ = self.stop();
        if let Some(runtime) = self.runtime.take() {
            runtime.shutdown_background();
        }
    }
}

async fn control_connection(
    shared: &SharedRef,
    connection: &mut Connection,
    command: &DeviceCommand,
    cancel: &mut watch::Receiver<bool>,
) -> Result<()> {
    let result = device::execute(connection, command, cancel, |event| {
        let mut s = lock(shared);
        match event {
            Progress::WriteAttempt {
                reference,
                persistent,
            } => {
                s.descriptor.device.valid = false;
                s.descriptor.metadata_revision += 1;
                if reference {
                    invalidate_reference(&mut s, "device_control");
                }
                if let Some(op) = &mut s.operation {
                    op.write_attempted = true;
                    if persistent {
                        op.persistence = "unverified".into();
                    }
                }
            }
            Progress::CommandSent => {
                if let Some(op) = &mut s.operation {
                    op.command_sent = true;
                }
            }
            Progress::RegisterVerified => {
                if let Some(op) = &mut s.operation {
                    op.register_verified = true;
                }
            }
            Progress::CompletionObserved => {
                if let Some(op) = &mut s.operation {
                    op.completion_observed = true;
                }
            }
        }
    })
    .await;
    match result {
        Ok((outcome, message, observation)) => {
            let mut s = lock(shared);
            if let Some(observation) = observation {
                s.descriptor.device = observation;
                s.descriptor.metadata_revision += 1;
            }
            s.status.configuration_report = Some(message.clone());
            if let Some(op) = &mut s.operation {
                op.outcome = outcome;
                op.message = Some(message);
            }
            Ok(())
        }
        Err(error) => {
            let mut s = lock(shared);
            if s.operation.as_ref().is_some_and(|op| op.write_attempted) {
                invalidate_reference(&mut s, "device_control_uncertain");
            }
            Err(error)
        }
    }
}

fn invalidate_reference(s: &mut Shared, reason: &str) {
    if !s
        .operation
        .as_ref()
        .is_some_and(|op| op.reference_may_have_changed)
    {
        s.descriptor.reference_epoch += 1;
    }
    s.descriptor.reference_reason = reason.into();
    s.descriptor.metadata_revision += 1;
    if let Some(op) = &mut s.operation {
        op.reference_may_have_changed = true;
    }
}

fn snapshot(shared: &SharedRef) -> Snapshot {
    let s = lock(shared);
    snapshot_at(&s, Instant::now())
}

fn pose_at(s: &Shared, now: Instant) -> Option<PoseSnapshot> {
    let mut pose = s.pose.clone()?;
    let received = s.last_pose?;
    pose.age_ns = now
        .duration_since(received)
        .as_nanos()
        .min(i64::MAX as u128) as u64;
    pose.fresh = fresh(s, now);
    Some(pose)
}

fn snapshot_at(s: &Shared, now: Instant) -> Snapshot {
    let is_fresh = fresh(s, now);
    let mut status = s.status.clone();
    status.battery = s.battery.snapshot(
        now,
        matches!(
            status.state,
            ConnectionState::Connecting
                | ConnectionState::Active
                | ConnectionState::Stale
                | ConnectionState::Complete
        ),
    );
    if status.state == ConnectionState::Active && !is_fresh {
        status.state = ConnectionState::Stale;
    }
    Snapshot {
        schema: SNAPSHOT_SCHEMA_VERSION,
        descriptor: s.descriptor.clone(),
        status,
        pose: pose_at(s, now),
        operation: s.operation.clone(),
    }
}

fn send_telemetry(shared: &SharedRef, telemetry: &mut Option<Telemetry>, force: bool) {
    if let Some(telemetry) = telemetry {
        let previous_errors = telemetry.send_errors();
        match telemetry.refresh(&snapshot(shared), force, Instant::now()) {
            Ok(count) => lock(shared).status.telemetry_sent += count,
            Err(error) => {
                let mut s = lock(shared);
                if telemetry.send_errors() == previous_errors {
                    s.status.send_errors += 1;
                }
                s.status.last_error = Some(error.to_string());
            }
        }
        lock(shared).status.send_errors += telemetry.send_errors() - previous_errors;
    }
}

fn begin_session(shared: &SharedRef) {
    let mut s = lock(shared);
    if s.descriptor.session_id != 0 {
        s.descriptor.reference_epoch += 1;
        s.descriptor.reference_reason = "reconnected".into();
        s.descriptor.device.valid = false;
    }
    s.motion = History::default();
    s.battery = Battery::default();
    s.pose = None;
    s.last_pose = None;
    s.first_pose = None;
    if s.descriptor.instance_id == 0 {
        s.descriptor.instance_id = new_id();
    }
    s.status.session_id = new_id();
    s.status.session_samples = 0;
    s.status.actual_rate_hz = 0.0;
    s.status.interval_min_ms = 0.0;
    s.status.interval_max_ms = 0.0;
    s.descriptor.session_id = s.status.session_id;
    s.descriptor.metadata_revision += 1;
    s.status.state = ConnectionState::Connecting;
}

struct PreparedPose {
    q: pose::Quat,
    raw: RawData,
    time: Option<SampleTime>,
    signals: Option<Signals>,
}
fn publish_batch(shared: &SharedRef, start: Instant, now: Instant, poses: Vec<PreparedPose>) {
    let mut s = lock(shared);
    s.motion.delivery += 1;
    for p in poses {
        if publish_locked(&mut s, p.q, &p.raw, start, now, p.time, p.signals).is_err() {
            s.status.invalid_poses += 1;
        }
    }
}
#[cfg(test)]
fn publish(
    shared: &SharedRef,
    q: pose::Quat,
    raw: &RawData,
    start: Instant,
    now: Instant,
    time: Option<SampleTime>,
) -> Result<()> {
    publish_locked(&mut lock(shared), q, raw, start, now, time, None)
}
fn publish_locked(
    s: &mut Shared,
    q: pose::Quat,
    raw: &RawData,
    start: Instant,
    now: Instant,
    sample_time: Option<SampleTime>,
    signals: Option<Signals>,
) -> Result<()> {
    let q = pose::normalize(q)?;
    let euler = pose::to_euler(q)?;
    if let Some(last) = s.last_pose {
        let gap = now.duration_since(last).as_secs_f64() * 1000.0;
        if s.status.session_samples == 1 || gap < s.status.interval_min_ms {
            s.status.interval_min_ms = gap;
        }
        s.status.interval_max_ms = s.status.interval_max_ms.max(gap);
    }
    let first = *s.first_pose.get_or_insert(now);
    s.last_pose = Some(now);
    s.status.pose_count += 1;
    s.status.session_samples += 1;
    let elapsed = now.duration_since(first).as_secs_f64();
    s.status.actual_rate_hz = if elapsed > 0.0 {
        (s.status.session_samples - 1) as f64 / elapsed
    } else {
        0.0
    };
    s.status.state = ConnectionState::Active;
    s.pose = Some(PoseSnapshot {
        instance_id: s.descriptor.instance_id,
        reference_epoch: s.descriptor.reference_epoch,
        metadata_revision: s.descriptor.metadata_revision,
        session_id: s.status.session_id,
        sequence: s.status.session_samples,
        received_ns: now.duration_since(start).as_nanos().min(i64::MAX as u128) as u64,
        age_ns: 0,
        sample_time,
        quaternion_xyzw: q,
        euler_deg: euler,
        raw: raw.clone(),
        fresh: true,
    });
    if let Some(signals) = signals {
        s.motion.push(
            now,
            MotionSample {
                source_id: s.descriptor.source_id.clone(),
                cursor: MotionCursor {
                    instance_id: s.descriptor.instance_id,
                    session_id: s.status.session_id,
                    sequence: s.status.session_samples,
                },
                reference_epoch: s.descriptor.reference_epoch,
                metadata_revision: s.descriptor.metadata_revision,
                delivery_id: s.motion.delivery,
                received_ns: now.duration_since(start).as_nanos().min(i64::MAX as u128) as u64,
                age_ns: 0,
                fresh: true,
                sample_time,
                orientation_xyzw: signals.orientation,
                euler_deg: euler,
                angular_velocity_rad_s: signals.gyro,
                acceleration_g: signals.acceleration,
                orientation_source: signals.source,
                profile: signals.profile,
            },
        );
    }
    Ok(())
}

fn emit(shared: &SharedRef, sender: &mut Option<Sender>) -> Result<()> {
    let (pose, source_id, now) = {
        let s = lock(shared);
        let now = Instant::now();
        (pose_at(&s, now), s.descriptor.source_id.clone(), now)
    };
    if let Some(sender) = sender {
        let previous_errors = sender.send_errors();
        let sent = if let Some(pose) = pose {
            sender.send_if_due(&pose, &source_id, pose.age_ns, now)?
        } else {
            sender.clear_pending();
            false
        };
        let mut s = lock(shared);
        s.status.osc_sent += u64::from(sent);
        s.status.coalesced_samples = sender.coalesced_samples();
        s.status.send_errors += sender.send_errors() - previous_errors;
    }
    Ok(())
}

async fn wait_for_deadline(deadline: Option<Instant>) {
    match deadline {
        Some(deadline) => tokio::time::sleep_until(deadline.into()).await,
        None => std::future::pending::<()>().await,
    }
}

fn simulated_angles(pattern: Pattern, mut angles: [f64; 3], t: f64) -> [f64; 3] {
    match pattern {
        Pattern::Fixed => {}
        Pattern::Yaw => angles[0] += 45.0 * (t * std::f64::consts::TAU * 0.25).sin(),
        Pattern::Combined => {
            angles[0] += 45.0 * (t * 1.5).sin();
            angles[1] += 20.0 * (t * 1.1).sin();
            angles[2] += 15.0 * (t * 0.9).sin();
        }
        Pattern::Wrap => angles[0] = (170.0 + t * 20.0 + 180.0).rem_euclid(360.0) - 180.0,
    }
    angles
}
fn signals(
    m: pose::Mounting,
    q: Result<pose::Quat>,
    gyro: Option<[f64; 3]>,
    acc: Option<[f64; 3]>,
    source: OrientationSource,
    profile: u8,
) -> Result<Signals> {
    Ok(Signals {
        orientation: q?,
        gyro: gyro
            .map(|v| m.vector(v).map(|v| v.map(f64::to_radians)))
            .transpose()?,
        acceleration: acc.map(|v| m.vector(v)).transpose()?,
        source,
        profile,
    })
}
#[derive(Default)]
struct FullFrameRate {
    last: Option<u64>,
    gaps: std::collections::VecDeque<u64>,
}
impl FullFrameRate {
    fn observe(&mut self, profile: u8, time: Option<u64>) -> Result<()> {
        if profile != 0xe4 {
            *self = Self::default();
            return Ok(());
        }
        let Some(time) = time else {
            return Err(Error::Protocol("full inertial frame lacks a clock".into()));
        };
        if let Some(previous) = self.last {
            if time < previous {
                self.gaps.clear();
            } else if time > previous {
                if self.gaps.len() == 8 {
                    self.gaps.pop_front();
                }
                self.gaps.push_back(time - previous);
                if self.gaps.len() >= 4 {
                    let mut sorted: Vec<_> = self.gaps.iter().copied().collect();
                    sorted.sort_unstable();
                    if !(45..=55).contains(&sorted[sorted.len() / 2]) {
                        return Err(Error::Protocol("full inertial frames are outside the 20 Hz validation range; select a short profile".into()));
                    }
                }
            }
        }
        self.last = Some(time);
        Ok(())
    }
}

async fn run(config: Config, shared: SharedRef, mut cancel: watch::Receiver<bool>) -> Result<()> {
    #[cfg(target_os = "windows")]
    let _timer_resolution = crate::timing::TimerResolution::for_config(&config)?;
    let mut sender = config.osc.clone().map(Sender::new).transpose()?;
    if let Source::Simulate {
        pattern,
        euler_deg,
        rate_hz,
        sample_clock,
    } = config.source
    {
        begin_session(&shared);
        let start = Instant::now();
        let mut clock = SampleClock::default();
        let mut sampling = tokio::time::interval(Duration::from_secs_f64(1.0 / rate_hz as f64));
        sampling.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
        loop {
            let output_deadline = sender.as_ref().and_then(Sender::deadline);
            tokio::select! {
                _ = cancel.changed() => return Err(Error::Cancelled),
                _ = sampling.tick() => {
                    let now = Instant::now();
                    let elapsed = now.duration_since(start);
                    let t = elapsed.as_secs_f64();
                    let angles = simulated_angles(pattern,euler_deg,t);
                    let sample_time = if sample_clock {
                        // A late tick followed by a prompt tick can share one millisecond.
                        // Do not manufacture a newer timestamp to disguise that duplicate.
                        let Some(time) = clock.observe(
                            SampleTimeKind::SimulatedElapsed,
                            elapsed.as_millis().min(i64::MAX as u128) as u64,
                            elapsed.as_nanos().min(i64::MAX as u128) as u64,
                        ) else {
                            lock(&shared).status.duplicate_sample_times = clock.duplicates;
                            continue;
                        };
                        Some(time)
                    } else { None };
                    let physical = pose::physical_from_euler(angles)?;
                    let next = pose::physical_from_euler(simulated_angles(pattern,euler_deg,t+0.0001))?;
                    let [x,y,z,w]=physical;
                    let delta=pose::multiply([-x,-y,-z,w],next);
                    let gyro=std::array::from_fn(|i| delta[i]*2.0/0.0001);
                    publish_batch(&shared,start,now,vec![PreparedPose {
                        q:pose::from_euler(angles)?,raw:RawData::default(),time:sample_time,
                        signals:Some(Signals {orientation:physical,gyro:Some(gyro),acceleration:None,source:OrientationSource::Simulator,profile:0}),
                    }]);
                    emit(&shared,&mut sender)?;
                },
                _ = wait_for_deadline(output_deadline) => emit(&shared,&mut sender)?,
            }
        }
    }
    let mut backoff = 1;
    loop {
        if *cancel.borrow() {
            return Err(Error::Cancelled);
        }
        lock(&shared).status.state = ConnectionState::Connecting;
        let attempt = Connection::open(&config.source, &mut cancel).await;
        let result = match attempt {
            Ok(mut connection) => {
                begin_session(&shared);
                let name = connection.device_name(&config.source).await;
                {
                    let mut state = lock(&shared);
                    if state.descriptor.device_name != name {
                        state.descriptor.device_name = name;
                        state.descriptor.metadata_revision += 1;
                    }
                }
                let result =
                    pump(&config, &shared, &mut connection, &mut sender, &mut cancel).await;
                connection.close().await;
                if lock(&shared).status.session_samples > 0 {
                    backoff = 1;
                }
                result
            }
            Err(e) => Err(e),
        };
        match result {
            Err(
                e @ (Error::Cancelled
                | Error::Permission(_)
                | Error::Protocol(_)
                | Error::Invalid(_)),
            ) => return Err(e),
            Err(e) => {
                let mut s = lock(&shared);
                s.status.last_error = Some(e.to_string());
                s.status.state = ConnectionState::Reconnecting;
                s.status.reconnect_count += 1;
            }
            Ok(()) => return Ok(()),
        }
        tokio::select! {
            _ = cancel.changed() => return Err(Error::Cancelled),
            _ = tokio::time::sleep(Duration::from_secs(backoff)) => {},
        }
        backoff = (backoff * 2).min(8);
    }
}

async fn pump(
    config: &Config,
    shared: &SharedRef,
    connection: &mut Connection,
    sender: &mut Option<Sender>,
    cancel: &mut watch::Receiver<bool>,
) -> Result<()> {
    let start = Instant::now();
    let (mut reader, writer) = connection.split();
    // A bounded serial writer is polled alongside reception. Dropping pump also
    // drops an in-progress write; no detached task can outlive the connection.
    let (requests, request_rx) = tokio::sync::mpsc::channel::<u16>(1);
    let writes =
        futures_util::stream::unfold((writer, request_rx), |(mut writer, mut rx)| async move {
            let address = rx.recv().await?;
            let result = tokio::time::timeout(
                Duration::from_secs(2),
                writer.write(&protocol::read_register(address)),
            )
            .await
            .unwrap_or_else(|_| {
                Err(Error::Timeout(format!(
                    "write read request 0x{address:02x}"
                )))
            });
            Some(((address, result), (writer, rx)))
        });
    tokio::pin!(writes);
    let mut writing = false;
    let mut next_battery_request = start + Duration::from_secs(1);
    let mut battery_outstanding: Option<Instant> = None;
    let mut last_bytes = start;
    let mut last_delivery: Option<Instant> = None;
    let mut next_link_check = start + Duration::from_secs(1);
    lock(shared).status.ble_link = reader.link_status();
    let mut parser = Parser::default();
    // Public frame counters span the whole start; this watchdog is connection-local.
    let mut received_frame = false;
    let mut raw = RawData::default();
    let mut sample_clock = SampleClock::default();
    let mut full_frame_rate = FullFrameRate::default();
    let mounting = config
        .mounting
        .ok_or_else(|| Error::Invalid("mounting required".into()))?;
    let mut next_request = start;
    let mut outstanding: Option<Instant> = None;
    // This timer only services health checks. OSC and optional quaternion reads
    // each use their own deadline, so maintenance cannot quantize their cadence.
    let mut ticks = tokio::time::interval(Duration::from_millis(20));
    ticks.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
    loop {
        let output_deadline = sender.as_ref().and_then(Sender::deadline);
        let request_deadline = (config.pose_input == PoseInput::Quaternion && !writing)
            .then(|| outstanding.map_or(next_request, |sent| sent + Duration::from_millis(250)));
        let battery_deadline =
            (!writing && battery_outstanding.is_none()).then_some(next_battery_request);
        let battery_timeout = battery_outstanding.map(|sent| sent + battery::RESPONSE_TIMEOUT);
        tokio::select! {
            _ = cancel.changed() => return Err(Error::Cancelled),
            bytes = reader.read() => {
                let bytes = bytes?;
                last_bytes = Instant::now();
                let ns = last_bytes.duration_since(start).as_nanos().min(i64::MAX as u128) as u64;
                let old_discarded = parser.discarded_bytes;
                let old_invalid = parser.invalid_frames;
                let frames = parser.push(&bytes);
                // Battery replies cannot validate an otherwise unsupported pose stream.
                received_frame |= frames.iter().any(|frame| !matches!(frame, Frame::Registers { address: battery::REGISTER, .. }));
                {
                    let mut state = lock(shared);
                    state.status.bytes_received += bytes.len() as u64;
                    let delivery = &mut state.status.delivery;
                    delivery.reads += 1;
                    delivery.max_bytes_per_read = delivery.max_bytes_per_read.max(bytes.len() as u64);
                    delivery.max_frames_per_read = delivery.max_frames_per_read.max(frames.len() as u64);
                    if let Some(previous) = last_delivery {
                        let gap = last_bytes.duration_since(previous).as_secs_f64() * 1000.0;
                        let bucket = [1.0, 10.0, 30.0, 100.0].partition_point(|limit| gap >= *limit);
                        delivery.gap_histogram[bucket] += 1;
                        delivery.max_gap_ms = delivery.max_gap_ms.max(gap);
                    }
                    last_delivery = Some(last_bytes);
                }
                let mut prepared = Vec::new();
                for frame in frames {
                    lock(shared).status.frames_received += 1;
                    let mut sample_time_ms = None;
                    let mut coherent = None;
                    let q = match frame {
                        Frame::Motion { acceleration_g,angular_velocity_dps,euler_xyz_deg } => {
                            raw.acceleration_g=Some(acceleration_g);raw.angular_velocity_dps=Some(angular_velocity_dps);
                            raw.euler_xyz_deg=Some(euler_xyz_deg);raw.motion_received_ns=Some(ns);
                            if config.pose_input==PoseInput::Euler {
                                coherent=Some(signals(mounting,mounting.physical_from_sensor_euler(euler_xyz_deg),Some(angular_velocity_dps),Some(acceleration_g),OrientationSource::ConvertedEuler,0x61));
                                Some(mounting.from_sensor_euler(euler_xyz_deg))
                            } else { None }
                        },
                        Frame::Stream { profile: flag, acceleration_g, sample_time_ms: time, angular_velocity_dps, euler_xyz_deg, quaternion_wxyz } => {
                            // Partial stream groups must not retain old fields under a new host timestamp.
                            full_frame_rate.observe(flag,time)?;
                            raw.acceleration_g = acceleration_g;
                            raw.angular_velocity_dps = angular_velocity_dps;
                            raw.euler_xyz_deg = euler_xyz_deg;
                            raw.motion_received_ns = (angular_velocity_dps.is_some() || euler_xyz_deg.is_some()).then_some(ns);
                            if let Some(q) = quaternion_wxyz {
                                raw.quaternion_wxyz = Some(q);
                                raw.quaternion_received_ns = Some(ns);
                            }
                            sample_time_ms = time;
                            match config.pose_input {
                                PoseInput::Euler => euler_xyz_deg.map(|e| {
                                    coherent=Some(signals(mounting,mounting.physical_from_sensor_euler(e),angular_velocity_dps,acceleration_g,OrientationSource::ConvertedEuler,flag));
                                    mounting.from_sensor_euler(e)
                                }),
                                PoseInput::StreamQuaternion => quaternion_wxyz.map(|[w,x,y,z]| {
                                    coherent=Some(signals(mounting,mounting.physical_from_sensor_quaternion([x,y,z,w]),angular_velocity_dps,acceleration_g,OrientationSource::NativeQuaternion,flag));
                                    mounting.from_sensor_quaternion([x,y,z,w])
                                }),
                                PoseInput::Quaternion => None,
                            }
                        },
                        Frame::Registers { address:0x51,values } => {
                            outstanding=None;
                            let q = [values[0],values[1],values[2],values[3]].map(|v| v as f64/32768.0);
                            raw.quaternion_wxyz=Some(q);raw.quaternion_received_ns=Some(ns);
                            if config.pose_input==PoseInput::Quaternion {
                                coherent=Some(signals(mounting,mounting.physical_from_sensor_quaternion([q[1],q[2],q[3],q[0]]),None,None,OrientationSource::RegisterQuaternion,0x71));
                                Some(mounting.from_sensor_quaternion([q[1],q[2],q[3],q[0]]))
                            } else { None }
                        },
                        Frame::Registers { address: battery::REGISTER, values } => {
                            if let Some(sent) = battery_outstanding.take() {
                                if last_bytes.duration_since(sent) < battery::RESPONSE_TIMEOUT {
                                    lock(shared).battery.observe(values[0] as u16, last_bytes);
                                } else {
                                    lock(shared).battery.fail("battery read timed out (register 0x64)".into());
                                }
                            }
                            None
                        },
                        _ => None,
                    };
                    if let Some(q) = q {
                        match q {
                            Ok(q) => {
                                let sample_time = if let Some(ms) = sample_time_ms {
                                    let old_discontinuities = sample_clock.discontinuities;
                                    let Some(time) = sample_clock.observe(SampleTimeKind::DeviceCalendar, ms, ns) else {
                                        lock(shared).status.duplicate_sample_times += 1;
                                        continue;
                                    };
                                    lock(shared).status.clock_discontinuities += sample_clock.discontinuities - old_discontinuities;
                                    Some(time)
                                } else { None };
                                match coherent.transpose() {
                                    Ok(signals) => prepared.push(PreparedPose { q,raw:raw.clone(),time:sample_time,signals }),
                                    Err(_) => lock(shared).status.invalid_poses += 1,
                                }
                            },
                            Err(_) => lock(shared).status.invalid_poses+=1,
                        }
                    }
                }
                {
                    let mut state = lock(shared);
                    state.status.discarded_bytes += parser.discarded_bytes - old_discarded;
                    state.status.invalid_frames += parser.invalid_frames - old_invalid;
                }
                publish_batch(shared,start,last_bytes,prepared);
                emit(shared,sender)?;
            },
            _ = wait_for_deadline(output_deadline) => emit(shared,sender)?,
            _ = wait_for_deadline(request_deadline) => {
                requests.try_send(0x51).map_err(|e| Error::Internal(e.to_string()))?;
                writing = true;
                let sent = Instant::now();
                outstanding=Some(sent);
                next_request=sent+Duration::from_millis(20);
            },
            _ = wait_for_deadline(battery_deadline) => {
                requests.try_send(battery::REGISTER).map_err(|e| Error::Internal(e.to_string()))?;
                writing = true;
                let sent = Instant::now();
                battery_outstanding = Some(sent);
                next_battery_request = sent + battery::POLL_INTERVAL;
            },
            Some((address, result)) = writes.next() => {
                writing = false;
                if address == battery::REGISTER {
                    if let Err(error) = result {
                        battery_outstanding = None;
                        lock(shared).battery.fail(error.to_string());
                    }
                } else {
                    result?;
                }
            },
            _ = wait_for_deadline(battery_timeout) => {
                battery_outstanding = None;
                lock(shared).battery.fail("battery read timed out (register 0x64)".into());
            },
            _ = ticks.tick() => {
                let now = Instant::now();
                if now >= next_link_check {
                    let link_status = reader.link_status();
                    lock(shared).status.ble_link = link_status;
                    next_link_check = now + Duration::from_secs(1);
                }
                if now.duration_since(start) >= STALE_AFTER {
                    let mut state = lock(shared);
                    if state.pose.is_none() || !fresh(&state, now) {
                        state.status.state = ConnectionState::Stale;
                    }
                }
                if now.duration_since(last_bytes)>Duration::from_secs(10) { return Err(Error::Timeout("no device bytes for 10 seconds".into())); }
                if start.elapsed()>Duration::from_secs(10) && !received_frame {
                    return Err(Error::Protocol("no supported WIT frames; check output format, model, firmware and baud rate".into()));
                }
            },
        }
    }
}

#[cfg(test)]
mod snapshot_tests {
    use super::*;

    #[test]
    fn battery_age_is_independent_and_new_sessions_clear_old_voltage() {
        let shared = Arc::new(Mutex::new(Shared::default()));
        begin_session(&shared);
        let now = Instant::now();
        let before = snapshot_at(&lock(&shared), now);
        lock(&shared).battery.observe(387, now);
        let reading = snapshot_at(&lock(&shared), now + Duration::from_secs(2));
        assert!(reading.pose.is_none());
        assert_eq!(reading.status.pose_count, 0);
        assert_eq!(
            reading.descriptor.metadata_revision,
            before.descriptor.metadata_revision
        );
        assert_eq!(
            reading.descriptor.reference_epoch,
            before.descriptor.reference_epoch
        );
        assert!(reading.status.battery.fresh);
        let json: serde_json::Value =
            serde_json::from_str(&snapshot_json(&reading).unwrap()).unwrap();
        assert_eq!(json["status"]["battery"]["voltage_v"], 3.87);
        assert_eq!(json["status"]["battery"]["estimated_percent"], 75);
        assert_eq!(json["status"]["battery"]["age_ns"], "2000000000");
        for state in [
            ConnectionState::Reconnecting,
            ConnectionState::Stopped,
            ConnectionState::Failed,
        ] {
            lock(&shared).status.state = state;
            let stopped = snapshot_at(&lock(&shared), now + Duration::from_secs(3));
            assert!(!stopped.status.battery.fresh);
            assert_eq!(stopped.status.battery.age_ns, Some(3_000_000_000));
        }
        begin_session(&shared);
        let reconnected = snapshot_at(&lock(&shared), now + Duration::from_secs(4));
        assert!(reconnected.status.battery.voltage_v.is_none());
        assert!(reconnected.status.battery.age_ns.is_none());
        assert!(!reconnected.status.battery.fresh);
    }

    #[test]
    fn query_age_freshness_and_stopped_pose_share_one_host_clock() {
        let shared = Arc::new(Mutex::new(Shared::default()));
        begin_session(&shared);
        let start = Instant::now();
        let received = start + Duration::from_millis(10);
        let sample = Some(SampleTime {
            kind: SampleTimeKind::DeviceCalendar,
            time_ms: 473398726930,
            clock_epoch: 1,
        });
        publish(
            &shared,
            [0., 0., 0., 1.],
            &RawData::default(),
            start,
            received,
            sample,
        )
        .unwrap();
        let original = snapshot_at(&lock(&shared), received).pose.unwrap();
        for age in [0, 1, 499_999_999, 500_000_000, 750_000_000] {
            let value = snapshot_at(&lock(&shared), received + Duration::from_nanos(age));
            let pose = value.pose.as_ref().unwrap();
            assert_eq!(pose.age_ns, age);
            assert_eq!(pose.fresh, age < 500_000_000);
            assert_eq!(
                value.status.state,
                if pose.fresh {
                    ConnectionState::Active
                } else {
                    ConnectionState::Stale
                }
            );
            assert_eq!(pose.sequence, original.sequence);
            assert_eq!(pose.received_ns, original.received_ns);
            assert_eq!(pose.metadata_revision, original.metadata_revision);
            assert_eq!(pose.sample_time, sample);
            let json: serde_json::Value =
                serde_json::from_str(&snapshot_json(&value).unwrap()).unwrap();
            assert_eq!(json["schema"], 4);
            assert_eq!(json["pose"]["age_ns"], age.to_string());
        }
        lock(&shared).status.state = ConnectionState::Stopped;
        for age in [1_000_000_000, 2_000_000_000] {
            let value = snapshot_at(&lock(&shared), received + Duration::from_nanos(age));
            let pose = value.pose.unwrap();
            assert!(!pose.fresh);
            assert_eq!(pose.age_ns, age);
            assert_eq!(pose.sequence, original.sequence);
            assert_eq!(value.status.state, ConnectionState::Stopped);
        }
    }

    #[test]
    fn new_sessions_clear_age_and_device_calendar_does_not_define_it() {
        let shared = Arc::new(Mutex::new(Shared::default()));
        let start = Instant::now();
        assert!(snapshot_at(&lock(&shared), start).pose.is_none());
        begin_session(&shared);
        for (index, time_ms) in [473398726930, 0].into_iter().enumerate() {
            let received = start + Duration::from_secs(index as u64);
            publish(
                &shared,
                [0., 0., 0., 1.],
                &RawData::default(),
                start,
                received,
                Some(SampleTime {
                    kind: SampleTimeKind::DeviceCalendar,
                    time_ms,
                    clock_epoch: index as u64 + 1,
                }),
            )
            .unwrap();
            let pose = snapshot_at(&lock(&shared), received + Duration::from_millis(20))
                .pose
                .unwrap();
            assert_eq!(pose.age_ns, 20_000_000);
            assert!(pose.fresh);
        }
        let old_session = lock(&shared).status.session_id;
        begin_session(&shared);
        let value = snapshot_at(&lock(&shared), start + Duration::from_secs(3));
        assert!(value.pose.is_none());
        assert_ne!(value.descriptor.session_id, old_session);
        let received = start + Duration::from_secs(4);
        publish(
            &shared,
            [0., 0., 0., 1.],
            &RawData::default(),
            received,
            received,
            None,
        )
        .unwrap();
        let pose = snapshot_at(&lock(&shared), received).pose.unwrap();
        assert_eq!(pose.age_ns, 0);
        assert_eq!(pose.sequence, 1);
        assert!(pose.fresh);
    }
}

#[cfg(all(test, unix))]
mod tests {
    use super::*;
    use crate::pose::Mounting;
    use std::net::UdpSocket;
    use tokio::io::{AsyncReadExt, AsyncWriteExt};
    use tokio_serial::SerialStream;

    async fn fake_device(
        algorithm: u16,
    ) -> (
        Connection,
        Arc<Mutex<Vec<[u8; 5]>>>,
        tokio::task::JoinHandle<()>,
    ) {
        let (mut device, host) = SerialStream::pair().unwrap();
        let commands = Arc::new(Mutex::new(Vec::new()));
        let seen = commands.clone();
        let task = tokio::spawn(async move {
            let mut registers = [0u16; 128];
            registers[3] = 9;
            registers[0x0e] = 0x84;
            registers[0x1f] = 4;
            registers[0x24] = algorithm;
            registers[0x2e] = 13115;
            let mut bytes = [0u8; 5];
            while device.read_exact(&mut bytes).await.is_ok() {
                seen.lock().unwrap().push(bytes);
                if bytes[2] == 0x27 {
                    let address = u16::from_le_bytes([bytes[3], bytes[4]]) as usize;
                    let mut reply = [0u8; 20];
                    reply[..4].copy_from_slice(&[0x55, 0x71, bytes[3], bytes[4]]);
                    for i in 0..8 {
                        reply[4 + 2 * i..6 + 2 * i]
                            .copy_from_slice(&registers[address + i].to_le_bytes());
                    }
                    if address == 1 && registers[1] == 1 {
                        registers[1] = 0;
                    }
                    device.write_all(&reply).await.unwrap();
                } else if bytes[2] != 0x69 {
                    let address = bytes[2] as usize;
                    let value = u16::from_le_bytes([bytes[3], bytes[4]]);
                    if address == 0 && value == 1 {
                        registers[3] = 6;
                        registers[0x0e] = 0x61;
                        registers[0x24] = 0;
                    } else if address == 1 && [4, 8].contains(&value) {
                        registers[1] = 0;
                    } else {
                        registers[address] = value;
                    }
                }
            }
        });
        (Connection::Usb(host), commands, task)
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn inspection_is_read_only_and_device_controls_have_explicit_effects() {
        let (mut connection, commands, server) = fake_device(0).await;
        let (_tx, mut cancel) = watch::channel(false);
        let observation = device::inspect(&mut connection, &mut cancel).await.unwrap();
        assert!(observation.valid);
        assert_eq!(observation.rate_hz, Some(100.));
        assert_eq!(observation.algorithm, Some(AlgorithmMode::NineAxis));
        assert_eq!(observation.firmware_version, Some("13115".into()));
        assert_eq!(
            commands
                .lock()
                .unwrap()
                .iter()
                .map(|c| c[2])
                .collect::<Vec<_>>(),
            vec![0x27; 4]
        );
        commands.lock().unwrap().clear();
        assert!(matches!(
            device::execute(
                &mut connection,
                &DeviceCommand::ZeroYaw,
                &mut cancel,
                |_| {}
            )
            .await,
            Err(Error::Invalid(_))
        ));
        assert_eq!(
            *commands.lock().unwrap(),
            vec![protocol::read_register(0x24)]
        );
        for command in [
            DeviceCommand::Algorithm {
                mode: AlgorithmMode::SixAxis,
            },
            DeviceCommand::ZeroYaw,
            DeviceCommand::AngleReference,
            DeviceCommand::ResetDefaults,
        ] {
            commands.lock().unwrap().clear();
            let shared = Arc::new(Mutex::new(Shared::default()));
            {
                let mut state = lock(&shared);
                state.descriptor.device = observation.clone();
                state.operation = Some(OperationStatus::new(format!("{command:?}")));
            }
            control_connection(&shared, &mut connection, &command, &mut cancel)
                .await
                .unwrap();
            tokio::time::sleep(Duration::from_millis(10)).await;
            let state = lock(&shared);
            let op = state.operation.as_ref().unwrap();
            assert!(op.command_sent && op.reference_may_have_changed);
            assert_eq!(state.descriptor.reference_epoch, 2);
            let writes: Vec<_> = commands
                .lock()
                .unwrap()
                .iter()
                .copied()
                .filter(|c| c[2] != 0x27 && c[2] != 0x69)
                .collect();
            match command {
                DeviceCommand::Algorithm { .. } => {
                    assert_eq!(writes, vec![protocol::write_register(0x24, 1)]);
                    assert!(op.register_verified);
                    assert!(!state.descriptor.device.valid);
                }
                DeviceCommand::ZeroYaw => {
                    assert_eq!(writes, vec![protocol::write_register(1, 4)]);
                    assert_eq!(op.outcome, OperationOutcome::Unverified);
                }
                DeviceCommand::AngleReference => {
                    assert_eq!(
                        writes,
                        vec![
                            protocol::write_register(1, 8),
                            protocol::write_register(0, 0)
                        ]
                    );
                    assert_eq!(op.persistence, "unverified");
                }
                DeviceCommand::ResetDefaults => {
                    assert_eq!(writes, vec![protocol::write_register(0, 1)]);
                    assert!(op.register_verified && state.descriptor.device.valid);
                    assert_eq!(state.descriptor.device.rate_hz, Some(10.));
                }
                _ => unreachable!(),
            }
        }
        server.abort();
        let _ = server.await;
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn experimental_output_rejects_unsupported_rate_before_any_write() {
        let (mut connection, commands, server) = fake_device(0).await;
        let (_tx, mut cancel) = watch::channel(false);
        let output = DeviceCommand::Output {
            format: OutputProfile::ExperimentalFullInertial20Hz,
        };
        assert!(
            device::execute(&mut connection, &output, &mut cancel, |_| {})
                .await
                .is_err()
        );
        assert!(commands.lock().unwrap().iter().all(|c| c[2] == 0x27));
        device::execute(
            &mut connection,
            &DeviceCommand::Rate { hz: 20 },
            &mut cancel,
            |_| {},
        )
        .await
        .unwrap();
        device::execute(&mut connection, &output, &mut cancel, |_| {})
            .await
            .unwrap();
        commands.lock().unwrap().clear();
        assert!(
            device::execute(
                &mut connection,
                &DeviceCommand::Rate { hz: 100 },
                &mut cancel,
                |_| {}
            )
            .await
            .is_err()
        );
        assert!(commands.lock().unwrap().iter().all(|c| c[2] == 0x27));
        server.abort();
        let _ = server.await;
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn cancellation_after_write_invalidates_reference_without_repeating_write() {
        let (mut device, host) = SerialStream::pair().unwrap();
        let mut connection = Connection::Usb(host);
        let mut controller = Controller::new().unwrap();
        {
            let mut state = lock(&controller.shared);
            state.status.state = ConnectionState::Configuring;
            state.operation = Some(OperationStatus::new("rate".into()));
            state.descriptor.device.valid = true;
        }
        controller.launch(None, move |shared, mut cancel| async move {
            control_connection(
                &shared,
                &mut connection,
                &DeviceCommand::Rate { hz: 100 },
                &mut cancel,
            )
            .await
        });
        let mut bytes = [0u8; 5];
        device.read_exact(&mut bytes).await.unwrap();
        assert_eq!(bytes, protocol::read_register(0x0e));
        let mut reply = [0u8; 20];
        reply[..6].copy_from_slice(&[0x55, 0x71, 0x0e, 0, 0x61, 0]);
        device.write_all(&reply).await.unwrap();
        device.read_exact(&mut bytes).await.unwrap();
        assert_eq!(bytes, protocol::UNLOCK);
        device.read_exact(&mut bytes).await.unwrap();
        assert_eq!(bytes, protocol::write_register(3, 9));
        controller.stop().unwrap();
        let s = controller.snapshot();
        assert_eq!(s.status.state, ConnectionState::Stopped);
        let operation = s.operation.unwrap();
        assert_eq!(operation.outcome, OperationOutcome::Cancelled);
        assert!(operation.write_attempted && operation.reference_may_have_changed);
        assert!(!s.descriptor.device.valid);
        assert_eq!(s.descriptor.reference_epoch, 2);
        assert_eq!(s.descriptor.reference_reason, "device_control_uncertain");
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn reset_observation_advances_metadata_revision() {
        let (mut device, host) = SerialStream::pair().unwrap();
        let mut connection = Connection::Usb(host);
        let (ready_tx, ready_rx) = tokio::sync::oneshot::channel();
        let (resume_tx, resume_rx) = tokio::sync::oneshot::channel();
        let server = tokio::spawn(async move {
            let mut registers = [0u16; 128];
            registers[3] = 9;
            registers[0x0e] = 0x84;
            registers[0x24] = 1;
            registers[0x2e] = 13115;
            let mut ready = Some(ready_tx);
            let mut resume = Some(resume_rx);
            let mut bytes = [0u8; 5];
            while device.read_exact(&mut bytes).await.is_ok() {
                if bytes[2] == 0x27 {
                    let address = u16::from_le_bytes([bytes[3], bytes[4]]) as usize;
                    if address == 0x2e
                        && let Some(ready) = ready.take()
                    {
                        // Hold the final inspection reply so both descriptor states
                        // can be observed without relying on polling a timing window.
                        ready.send(()).unwrap();
                        resume.take().unwrap().await.unwrap();
                    }
                    let mut reply = [0u8; 20];
                    reply[..4].copy_from_slice(&[0x55, 0x71, bytes[3], bytes[4]]);
                    for i in 0..8 {
                        reply[4 + 2 * i..6 + 2 * i]
                            .copy_from_slice(&registers[address + i].to_le_bytes());
                    }
                    device.write_all(&reply).await.unwrap();
                } else if bytes == protocol::write_register(0, 1) {
                    registers[3] = 6;
                    registers[0x0e] = 0x61;
                    registers[0x24] = 0;
                }
            }
        });
        let shared = Arc::new(Mutex::new(Shared::default()));
        lock(&shared).operation = Some(OperationStatus::new("reset_defaults".into()));
        let observed = shared.clone();
        let (_tx, mut cancel) = watch::channel(false);
        let worker = tokio::spawn(async move {
            control_connection(
                &shared,
                &mut connection,
                &DeviceCommand::ResetDefaults,
                &mut cancel,
            )
            .await
        });
        tokio::time::timeout(Duration::from_secs(2), ready_rx)
            .await
            .unwrap()
            .unwrap();
        let before = snapshot(&observed).descriptor;
        resume_tx.send(()).unwrap();
        worker.await.unwrap().unwrap();
        let after = snapshot(&observed).descriptor;
        server.abort();
        let _ = server.await;
        assert!(!before.device.valid && after.device.valid);
        assert_eq!(after.device.rate_hz, Some(10.));
        assert!(after.metadata_revision > before.metadata_revision);
        assert_eq!(after.reference_epoch, before.reference_epoch);
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn unsupported_stream_after_reconnect_still_times_out() {
        let config = Config {
            source: Source::Usb {
                port: "test-pty".into(),
                baud: 115200,
            },
            mounting: Some(Mounting::parse("+x,+y,+z").unwrap()),
            ..Config::default()
        };
        let shared = Arc::new(Mutex::new(Shared::default()));
        begin_session(&shared);
        let first_config = config.clone();
        let first_shared = shared.clone();
        let (mut device, host) = SerialStream::pair().unwrap();
        let (_tx, mut cancel) = watch::channel(false);
        let first = tokio::spawn(async move {
            let mut connection = Connection::Usb(host);
            pump(
                &first_config,
                &first_shared,
                &mut connection,
                &mut None,
                &mut cancel,
            )
            .await
        });
        let mut frame = [0u8; 20];
        frame[..2].copy_from_slice(&[0x55, 0x61]);
        device.write_all(&frame).await.unwrap();
        let deadline = Instant::now() + Duration::from_secs(2);
        while lock(&shared).status.pose_count == 0 {
            assert!(Instant::now() < deadline);
            tokio::time::sleep(Duration::from_millis(2)).await;
        }
        drop(device);
        assert!(
            tokio::time::timeout(Duration::from_secs(2), first)
                .await
                .unwrap()
                .unwrap()
                .is_err()
        );

        begin_session(&shared);
        assert_eq!(lock(&shared).status.frames_received, 1);
        let (mut device, host) = SerialStream::pair().unwrap();
        let writer = tokio::spawn(async move {
            let mut tick = tokio::time::interval(Duration::from_millis(100));
            let mut command = [0u8; 5];
            let mut used = 0;
            loop {
                tokio::select! {
                    n = device.read(&mut command[used..]) => {
                        let n = n.unwrap();
                        assert!(n > 0);
                        used += n;
                        if used == command.len() {
                            assert_eq!(command, protocol::read_register(battery::REGISTER));
                            let mut response = [0u8; 20];
                            response[..4].copy_from_slice(&[0x55, 0x71, 0x64, 0]);
                            response[4..6].copy_from_slice(&382u16.to_le_bytes());
                            device.write_all(&response).await.unwrap();
                            used = 0;
                        }
                    },
                    _ = tick.tick() => { device.write_all(&[0u8; 16]).await.unwrap(); },
                }
            }
        });
        let mut connection = Connection::Usb(host);
        let (_tx, mut cancel) = watch::channel(false);
        let result = tokio::time::timeout(
            Duration::from_secs(12),
            pump(&config, &shared, &mut connection, &mut None, &mut cancel),
        )
        .await;
        writer.abort();
        let _ = writer.await;
        assert!(matches!(result, Ok(Err(Error::Protocol(_)))), "{result:?}");
        let s = snapshot(&shared);
        assert_eq!(s.status.frames_received, 2);
        assert_eq!(s.status.battery.voltage_v, Some(3.82));
        assert_eq!(s.status.pose_count, 1);
        assert_eq!(s.status.session_samples, 0);
        assert!(s.status.bytes_received > frame.len() as u64);
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn timestamp_stream_duplicates_stale_reset_and_absent_metadata() {
        let (mut device, host) = SerialStream::pair().unwrap();
        let mut connection = Connection::Usb(host);
        let config = Config {
            source: Source::Usb {
                port: "test-pty".into(),
                baud: 115200,
            },
            pose_input: PoseInput::StreamQuaternion,
            mounting: Some(Mounting::parse("+x,+y,+z").unwrap()),
            ..Config::default()
        };
        let shared = Arc::new(Mutex::new(Shared::default()));
        begin_session(&shared);
        let observed = shared.clone();
        let (tx, mut cancel) = watch::channel(false);
        let handle = tokio::spawn(async move {
            pump(&config, &shared, &mut connection, &mut None, &mut cancel).await
        });
        let frame = |ms| {
            vec![
                0x55, 0x84, 15, 1, 1, 0, 0, 0, ms, 0, 0xff, 0x7f, 0, 0, 0, 0, 0, 0,
            ]
        };
        device
            .write_all(&[frame(10), frame(15)].concat())
            .await
            .unwrap();
        let deadline = Instant::now() + Duration::from_secs(1);
        while lock(&observed).status.pose_count < 2 {
            assert!(Instant::now() < deadline);
            tokio::time::sleep(Duration::from_millis(2)).await;
        }
        let first = lock(&observed).pose.clone().unwrap();
        assert_eq!(first.sample_time.unwrap().clock_epoch, 1);
        assert_eq!(Some(first.received_ns), first.raw.quaternion_received_ns);
        assert_eq!(first.raw.euler_xyz_deg, None);
        // Duplicates arriving throughout the timeout cannot refresh the pose.
        for _ in 0..12 {
            device.write_all(&frame(15)).await.unwrap();
            tokio::time::sleep(Duration::from_millis(50)).await;
        }
        {
            let state = lock(&observed);
            assert_eq!(state.status.pose_count, 2);
            assert_eq!(state.status.duplicate_sample_times, 12);
            assert!(!fresh(&state, Instant::now()));
            assert_eq!(state.pose.as_ref().unwrap().received_ns, first.received_ns);
        }
        device.write_all(&frame(5)).await.unwrap();
        tokio::time::sleep(Duration::from_millis(30)).await;
        {
            let state = lock(&observed);
            assert!(fresh(&state, Instant::now()));
            assert_eq!(state.status.clock_discontinuities, 1);
            assert_eq!(
                state
                    .pose
                    .as_ref()
                    .unwrap()
                    .sample_time
                    .unwrap()
                    .clock_epoch,
                2
            );
        }
        device
            .write_all(&[0x55, 0x04, 0xff, 0x7f, 0, 0, 0, 0, 0, 0])
            .await
            .unwrap();
        tokio::time::sleep(Duration::from_millis(30)).await;
        assert!(lock(&observed).pose.as_ref().unwrap().sample_time.is_none());
        // Stream quaternion input does not poll quaternion registers. The first
        // low-frequency battery request is only due one second after connection.
        let mut command = [0u8; 5];
        assert!(
            tokio::time::timeout(Duration::from_millis(30), device.read_exact(&mut command))
                .await
                .is_err()
        );
        tx.send(true).unwrap();
        assert!(matches!(handle.await.unwrap(), Err(Error::Cancelled)));
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn quaternion_reads_wait_for_response_retry_timeout_and_cancel() {
        let (mut device, host) = SerialStream::pair().unwrap();
        let mut connection = Connection::Usb(host);
        let config = Config {
            source: Source::Usb {
                port: "test-pty".into(),
                baud: 115200,
            },
            pose_input: PoseInput::Quaternion,
            mounting: Some(Mounting::parse("+x,+y,+z").unwrap()),
            ..Config::default()
        };
        let shared = Arc::new(Mutex::new(Shared::default()));
        begin_session(&shared);
        let observed = shared.clone();
        let (tx, mut cancel) = watch::channel(false);
        let handle = tokio::spawn(async move {
            pump(&config, &shared, &mut connection, &mut None, &mut cancel).await
        });
        let mut command = [0u8; 5];
        tokio::time::timeout(Duration::from_secs(1), device.read_exact(&mut command))
            .await
            .unwrap()
            .unwrap();
        assert_eq!(command, protocol::read_register(0x51));
        assert!(
            tokio::time::timeout(Duration::from_millis(50), device.read_exact(&mut command))
                .await
                .is_err(),
            "more than one request in flight"
        );
        // Withhold the first response: retry must use its timeout, not a fast poll loop.
        tokio::time::timeout(Duration::from_millis(500), device.read_exact(&mut command))
            .await
            .unwrap()
            .unwrap();
        assert_eq!(command, protocol::read_register(0x51));
        let mut response = [0u8; 20];
        response[..4].copy_from_slice(&[0x55, 0x71, 0x51, 0]);
        response[4..6].copy_from_slice(&32767i16.to_le_bytes());
        // Same read, but different frames: even an identical host receipt time
        // cannot associate stream gyro/timestamp with a register quaternion.
        let stream = [
            0x55, 0xa4, 15, 1, 1, 0, 0, 0, 5, 0, 1, 0, 2, 0, 3, 0, 0xff, 0x7f, 0, 0, 0, 0, 0, 0,
        ];
        device
            .write_all(&[stream.as_slice(), response.as_slice()].concat())
            .await
            .unwrap();
        tokio::time::timeout(Duration::from_millis(200), device.read_exact(&mut command))
            .await
            .unwrap()
            .unwrap();
        assert_eq!(lock(&observed).status.pose_count, 1);
        assert!(lock(&observed).pose.as_ref().unwrap().sample_time.is_none());
        {
            let state = lock(&observed);
            let batch = state.motion.since(None, Instant::now(), true);
            assert_eq!(batch.samples.len(), 1);
            let sample = &batch.samples[0];
            assert_eq!(
                sample.orientation_source,
                OrientationSource::RegisterQuaternion
            );
            assert!(sample.sample_time.is_none());
            assert!(sample.angular_velocity_rad_s.is_none());
            assert!(sample.acceleration_g.is_none());
            assert!(
                state
                    .pose
                    .as_ref()
                    .unwrap()
                    .raw
                    .angular_velocity_dps
                    .is_some()
            );
        }
        assert!(fresh(&lock(&observed), Instant::now()));
        tx.send(true).unwrap();
        assert!(matches!(
            tokio::time::timeout(Duration::from_millis(200), handle)
                .await
                .unwrap()
                .unwrap(),
            Err(Error::Cancelled)
        ));
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn battery_reply_shares_parser_without_creating_poses_or_quaternion_retries() {
        let (mut device, host) = SerialStream::pair().unwrap();
        let mut connection = Connection::Usb(host);
        let config = Config {
            source: Source::Usb {
                port: "test-pty".into(),
                baud: 115200,
            },
            pose_input: PoseInput::Quaternion,
            mounting: Some(Mounting::parse("+x,+y,+z").unwrap()),
            ..Config::default()
        };
        let shared = Arc::new(Mutex::new(Shared::default()));
        begin_session(&shared);
        let observed = shared.clone();
        let (tx, mut cancel) = watch::channel(false);
        let worker = tokio::spawn(async move {
            pump(&config, &shared, &mut connection, &mut None, &mut cancel).await
        });
        let mut command = [0u8; 5];
        // Leave quaternion requests unanswered until the battery query arrives.
        tokio::time::timeout(Duration::from_secs(3), async {
            loop {
                device.read_exact(&mut command).await.unwrap();
                if command == protocol::read_register(battery::REGISTER) {
                    break;
                }
                assert_eq!(command, protocol::read_register(0x51));
            }
        })
        .await
        .unwrap();
        let mut response = [0u8; 20];
        response[..4].copy_from_slice(&[0x55, 0x71, 0x64, 0]);
        response[4..6].copy_from_slice(&382u16.to_le_bytes());
        // Fragmented reply followed by a complete quaternion in the same read.
        device.write_all(&response[..7]).await.unwrap();
        let mut quaternion = [0u8; 20];
        quaternion[..4].copy_from_slice(&[0x55, 0x71, 0x51, 0]);
        quaternion[4..6].copy_from_slice(&32767u16.to_le_bytes());
        device
            .write_all(&[&response[7..], &quaternion].concat())
            .await
            .unwrap();
        tokio::time::timeout(Duration::from_secs(1), async {
            while snapshot(&observed).status.pose_count == 0 {
                tokio::time::sleep(Duration::from_millis(5)).await;
            }
        })
        .await
        .unwrap();
        let result = snapshot(&observed);
        assert_eq!(result.status.pose_count, 1);
        assert_eq!(result.status.battery.voltage_v, Some(3.82));
        assert_eq!(result.status.battery.estimated_percent, Some(60));
        assert!(result.status.battery.fresh);
        assert_eq!(result.status.discarded_bytes, 0);
        assert_eq!(result.status.invalid_frames, 0);
        // Extra unsolicited register replies cannot replace the requested reading.
        response[4..6].copy_from_slice(&420u16.to_le_bytes());
        device.write_all(&response).await.unwrap();
        tokio::time::sleep(Duration::from_millis(30)).await;
        assert_eq!(snapshot(&observed).status.battery.voltage_v, Some(3.82));
        tx.send(true).unwrap();
        assert!(matches!(worker.await.unwrap(), Err(Error::Cancelled)));
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn missing_battery_reply_does_not_interrupt_pose_reception_or_retry_flood() {
        let (mut device, host) = SerialStream::pair().unwrap();
        let config = Config {
            source: Source::Usb {
                port: "test-pty".into(),
                baud: 115200,
            },
            mounting: Some(Mounting::parse("+x,+y,+z").unwrap()),
            ..Config::default()
        };
        let shared = Arc::new(Mutex::new(Shared::default()));
        begin_session(&shared);
        let observed = shared.clone();
        let (tx, mut cancel) = watch::channel(false);
        let worker = tokio::spawn(async move {
            pump(
                &config,
                &shared,
                &mut Connection::Usb(host),
                &mut None,
                &mut cancel,
            )
            .await
        });
        let mut request_count = 0;
        let mut command = [0u8; 5];
        let mut frame = [0u8; 20];
        frame[..2].copy_from_slice(&[0x55, 0x61]);
        let mut ticks = tokio::time::interval(Duration::from_millis(10));
        let mut sent = 0;
        tokio::time::timeout(Duration::from_secs(6), async {
            loop {
                tokio::select! {
                    result = device.read_exact(&mut command) => {
                        result.unwrap();
                        assert_eq!(command, protocol::read_register(battery::REGISTER));
                        request_count += 1;
                    },
                    _ = ticks.tick() => {
                        device.write_all(&frame).await.unwrap();
                        sent += 1;
                        if snapshot(&observed).status.battery.last_error.is_some() { break; }
                    },
                }
            }
            while snapshot(&observed).status.pose_count < sent {
                tokio::time::sleep(Duration::from_millis(5)).await;
            }
        })
        .await
        .unwrap();
        let result = snapshot(&observed);
        assert_eq!(request_count, 1);
        assert!(sent >= 300);
        assert_eq!(result.status.pose_count, sent);
        assert_eq!(result.status.state, ConnectionState::Active);
        assert_eq!(result.status.reconnect_count, 0);
        assert!(result.status.last_error.is_none());
        assert!(result.status.battery.voltage_v.is_none());
        assert!(!result.status.battery.fresh);
        assert!(
            result
                .status
                .battery
                .last_error
                .unwrap()
                .contains("timed out")
        );
        tx.send(true).unwrap();
        assert!(matches!(worker.await.unwrap(), Err(Error::Cancelled)));
    }

    // macOS PTYs do not implement IOSSIOSPEED. Use the library's PTY constructor instead
    // of weakening real-port configuration error handling to make a fake port open.
    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn usb_pty_pump_stale_recovery_and_disconnect() {
        let (mut device, host) = SerialStream::pair().unwrap();
        let mut connection = Connection::Usb(host);
        let socket = UdpSocket::bind("127.0.0.1:0").unwrap();
        socket.set_nonblocking(true).unwrap();
        let config = Config {
            source: Source::Usb {
                port: "test-pty".into(),
                baud: 115200,
            },
            mounting: Some(Mounting::parse("-y,+x,+z").unwrap()),
            osc: Some(OscConfig {
                target: socket.local_addr().unwrap(),
                max_rate_hz: 50,
                format: OscFormat::Euler,
            }),
            ..Config::default()
        };
        let shared = Arc::new(Mutex::new(Shared::default()));
        begin_session(&shared);
        let observed = shared.clone();
        let mut sender = config.osc.clone().map(Sender::new).transpose().unwrap();
        let (_tx, mut cancel) = watch::channel(false);
        let handle = tokio::spawn(async move {
            pump(&config, &shared, &mut connection, &mut sender, &mut cancel).await
        });
        tokio::time::sleep(Duration::from_millis(550)).await;
        assert_eq!(lock(&observed).status.state, ConnectionState::Stale);
        assert!(lock(&observed).pose.is_none());
        let frame = [
            0x55, 0x61, 0x11, 0, 0x1b, 0, 0x23, 8, 0, 0, 0, 0, 0, 0, 0x8b, 0, 0x9f, 0xff, 0x89,
            0xeb,
        ];
        for _ in 0..20 {
            device.write_all(&frame[..7]).await.unwrap();
            device.write_all(&frame[7..]).await.unwrap();
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
        tokio::time::sleep(Duration::from_millis(600)).await;
        assert_eq!(lock(&observed).status.state, ConnectionState::Stale);
        assert!(!fresh(&lock(&observed), Instant::now()));
        let mut bytes = [0u8; 512];
        let mut received = 0;
        while let Ok(n) = socket.recv(&mut bytes) {
            let (_, packet) = rosc::decoder::decode_udp(&bytes[..n]).unwrap();
            let rosc::OscPacket::Message(message) = packet else {
                panic!("wrong OSC packet");
            };
            assert_eq!(message.addr, "/posebridge/euler");
            let expected = [
                -5239.0 / 32768.0 * 180.0,
                97.0 / 32768.0 * 180.0,
                139.0 / 32768.0 * 180.0,
            ];
            for (arg, value) in message.args.iter().skip(13).zip(expected) {
                let rosc::OscType::Float(actual) = arg else {
                    panic!("wrong type");
                };
                assert!((*actual as f64 - value).abs() < 1e-4);
            }
            received += 1;
        }
        assert!(received > 3);
        tokio::time::sleep(Duration::from_millis(50)).await;
        assert!(
            socket.recv(&mut bytes).is_err(),
            "stale pose was retransmitted"
        );
        device.write_all(&frame).await.unwrap();
        tokio::time::sleep(Duration::from_millis(40)).await;
        assert_eq!(lock(&observed).status.state, ConnectionState::Active);
        assert_eq!(lock(&observed).status.pose_count, 21);
        drop(device);
        let ended = tokio::time::timeout(Duration::from_secs(2), handle)
            .await
            .unwrap()
            .unwrap();
        assert!(
            ended.is_err(),
            "physical EOF must leave the pump for reconnect"
        );
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn configuration_readback_retries_only_read_requests() {
        let (mut device, host) = SerialStream::pair().unwrap();
        let mut connection = Connection::Usb(host);
        let (tx, mut cancel) = watch::channel(false);
        let worker = tokio::spawn(async move {
            device::read_registers(&mut connection, 0x0e, Some(0x84), &mut cancel).await
        });
        let mut command = [0u8; 5];
        for _ in 0..2 {
            tokio::time::timeout(Duration::from_secs(1), device.read_exact(&mut command))
                .await
                .unwrap()
                .unwrap();
            assert_eq!(command, protocol::read_register(0x0e));
        }
        let mut response = [0u8; 20];
        response[..6].copy_from_slice(&[0x55, 0x71, 0x0e, 0, 0x81, 0]);
        device.write_all(&response).await.unwrap();
        tokio::time::timeout(Duration::from_secs(1), device.read_exact(&mut command))
            .await
            .unwrap()
            .unwrap();
        assert_eq!(command, protocol::read_register(0x0e));
        response[4] = 0x84;
        device.write_all(&response).await.unwrap();
        assert_eq!(worker.await.unwrap().unwrap()[0], 0x84);
        drop(tx);
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn output_profile_requires_new_firmware_and_matching_readback() {
        for (current, wrong) in [(0x61u16, false), (0x61, true), (0x01, false)] {
            let (mut device, host) = SerialStream::pair().unwrap();
            let mut connection = Connection::Usb(host);
            let commands = Arc::new(Mutex::new(Vec::new()));
            let seen = commands.clone();
            let server = tokio::spawn(async move {
                let mut reads = 0;
                let mut bytes = [0u8; 5];
                while device.read_exact(&mut bytes).await.is_ok() {
                    seen.lock().unwrap().push(bytes);
                    if bytes[2] == 0x27 {
                        assert_eq!(bytes, protocol::read_register(0x0e));
                        reads += 1;
                        let observed = if reads == 1 || wrong { current } else { 0x84 };
                        let mut reply = [0u8; 20];
                        reply[..4].copy_from_slice(&[0x55, 0x71, 0x0e, 0]);
                        reply[4..6].copy_from_slice(&observed.to_le_bytes());
                        device.write_all(&reply).await.unwrap();
                    }
                }
            });
            let (_tx, mut cancel) = watch::channel(false);
            let result = device::execute(
                &mut connection,
                &DeviceCommand::Output {
                    format: OutputProfile::TimestampQuaternion,
                },
                &mut cancel,
                |_| {},
            )
            .await;
            assert_eq!(result.is_err(), current == 1 || wrong);
            let sent = commands.lock().unwrap().clone();
            assert_eq!(sent[0], protocol::read_register(0x0e));
            if current == 1 {
                assert_eq!(sent.len(), 1, "ambiguous firmware must receive no writes");
            } else {
                assert_eq!(sent[1], protocol::UNLOCK);
                assert_eq!(sent[2], protocol::write_register(0x0e, 0x84));
                assert!(sent.len() >= 4);
                assert!(
                    sent[3..]
                        .iter()
                        .all(|command| *command == protocol::read_register(0x0e)),
                    "verification may repeat reads only; no implicit write or save"
                );
            }
            server.abort();
            let _ = server.await;
        }
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn usb_configuration_readback_mismatch_and_no_implicit_save() {
        for (command, wrong) in [
            (DeviceCommand::Rate { hz: 100 }, false),
            (DeviceCommand::Rate { hz: 100 }, true),
            (DeviceCommand::MagStart, false),
            (DeviceCommand::MagStop, false),
            (DeviceCommand::AccelCalibrate, false),
            (DeviceCommand::Save, false),
        ] {
            let (mut device, host) = SerialStream::pair().unwrap();
            let mut connection = Connection::Usb(host);
            let (address, value) = protocol::command_register(&command).unwrap();
            let accel = matches!(command, DeviceCommand::AccelCalibrate);
            let commands = Arc::new(Mutex::new(Vec::new()));
            let seen = commands.clone();
            let server = tokio::spawn(async move {
                let mut reads = 0;
                let mut bytes = [0u8; 5];
                while device.read_exact(&mut bytes).await.is_ok() {
                    seen.lock().unwrap().push(bytes);
                    if bytes[2] == 0x27 {
                        if bytes[3] == 0x0e {
                            let mut reply = [0u8; 20];
                            reply[..6].copy_from_slice(&[0x55, 0x71, 0x0e, 0, 0x61, 0]);
                            device.write_all(&reply).await.unwrap();
                            continue;
                        }
                        reads += 1;
                        assert_eq!(bytes[3], address);
                        let observed = if wrong {
                            value + 1
                        } else if accel && reads > 1 {
                            0
                        } else {
                            value
                        };
                        let mut response = [0u8; 20];
                        response[..4].copy_from_slice(&[0x55, 0x71, address, 0]);
                        response[4..6].copy_from_slice(&observed.to_le_bytes());
                        device.write_all(&response).await.unwrap();
                    }
                }
            });
            let (_tx, mut cancel) = watch::channel(false);
            let result = device::execute(&mut connection, &command, &mut cancel, |_| {}).await;
            tokio::time::sleep(Duration::from_millis(10)).await;
            assert_eq!(result.is_err(), wrong, "{result:?}");
            let sent = commands.lock().unwrap().clone();
            let writes: Vec<_> = sent.iter().filter(|c| c[2] != 0x27).copied().collect();
            assert_eq!(writes[0], protocol::UNLOCK);
            assert_eq!(writes[1], protocol::write_register(address, value));
            if address != 0 {
                assert!(sent.iter().all(|c| c[2] != 0), "implicit SAVE");
            }
            server.abort();
            let _ = server.await;
        }
    }
}

#[cfg(test)]
mod motion_tests {
    use super::*;
    #[test]
    fn batch_is_atomic_and_coherent_fields_do_not_use_legacy_cache() {
        let shared = Arc::new(Mutex::new(Shared::default()));
        begin_session(&shared);
        let now = Instant::now();
        let input = |gyro| PreparedPose {
            q: [0., 0., 0., 1.],
            raw: RawData {
                angular_velocity_dps: Some([999.; 3]),
                ..RawData::default()
            },
            time: None,
            signals: Some(Signals {
                orientation: [0., 0., 0., 1.],
                gyro,
                acceleration: None,
                source: OrientationSource::NativeQuaternion,
                profile: 0xa4,
            }),
        };
        publish_batch(
            &shared,
            now,
            now,
            vec![input(Some([1., 2., 3.])), input(None)],
        );
        let state = lock(&shared);
        let b = state.motion.since(None, now, true);
        assert_eq!(b.samples.len(), 2);
        assert_eq!(b.samples[0].delivery_id, b.samples[1].delivery_id);
        assert_eq!(b.samples[0].angular_velocity_rad_s, Some([1., 2., 3.]));
        assert_eq!(b.samples[1].angular_velocity_rad_s, None);
        assert_eq!(
            state.pose.as_ref().unwrap().sequence,
            b.cursor.unwrap().sequence
        );
        assert_eq!(
            state.pose.as_ref().unwrap().raw.angular_velocity_dps,
            Some([999.; 3])
        );
    }
    #[test]
    fn full_frame_guard_refuses_high_rate_and_allows_clock_restart() {
        let mut rate = FullFrameRate::default();
        for t in [0, 50, 100, 150, 200, 250] {
            rate.observe(0xe4, Some(t)).unwrap();
        }
        rate.observe(0xe4, Some(0)).unwrap();
        for t in [5, 10, 15] {
            rate.observe(0xe4, Some(t)).unwrap();
        }
        assert!(rate.observe(0xe4, Some(20)).is_err());
    }
}
