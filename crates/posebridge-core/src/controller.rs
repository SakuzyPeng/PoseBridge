use crate::device::{self, Progress};
use crate::model::*;
use crate::osc::{Sender, Telemetry};
use crate::pose;
use crate::protocol::{self, Frame, Parser};
use crate::sample_clock::SampleClock;
use crate::transport::{self, Connection};
use futures_util::FutureExt;
use std::future::Future;
use std::panic::AssertUnwindSafe;
use std::sync::{Arc, Mutex, MutexGuard, mpsc};
use std::time::{Duration, Instant};
use tokio::runtime::Runtime;
use tokio::sync::watch;

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
            s.pose = None;
            s.last_pose = None;
            s.status = StatusSnapshot::default();
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
            s.status.state = ConnectionState::Connecting;
            s.descriptor.instance_id = new_id();
            s.descriptor.session_id = 0;
            s.descriptor.metadata_revision += 1;
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
            s.status.last_error = None;
            s.status.configuration_report = None;
            s.descriptor.device.valid = false;
            s.descriptor.metadata_revision += 1;
            let mut operation = OperationStatus::new("inspect".into());
            operation.source_id = Some(s.descriptor.source_id.clone());
            s.operation = Some(operation);
        }
        self.launch(None, move |shared, mut cancel| async move {
            let mut connection = Connection::open(&source, &mut cancel).await?;
            let name = connection.device_name(&source).await;
            let result = device::inspect(&mut connection, &mut cancel).await;
            connection.close().await;
            let observation = result?;
            let mut s = lock(&shared);
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
        if let Some(task) = self.task.take() {
            let _ = task.cancel.send(true);
            if task.done.recv_timeout(Duration::from_secs(5)).is_err() {
                task.handle.abort();
                let _ = task.done.recv_timeout(Duration::from_secs(1));
                let mut s = lock(&self.shared);
                s.status.state = ConnectionState::Failed;
                s.status.last_error = Some("stop timed out; worker aborted".into());
                return Err(Error::Timeout("stop; worker aborted".into()));
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
    let is_fresh = fresh(&s, Instant::now());
    let mut status = s.status.clone();
    if status.state == ConnectionState::Active && !is_fresh {
        status.state = ConnectionState::Stale;
    }
    Snapshot {
        schema: PROTOCOL_VERSION,
        descriptor: s.descriptor.clone(),
        status,
        pose: s.pose.clone().map(|mut p| {
            p.fresh = is_fresh;
            p
        }),
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

fn publish(
    shared: &SharedRef,
    q: pose::Quat,
    raw: &RawData,
    start: Instant,
    now: Instant,
    sample_time: Option<SampleTime>,
) -> Result<()> {
    let q = pose::normalize(q)?;
    let euler = pose::to_euler(q)?;
    let mut s = lock(shared);
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
        sample_time,
        quaternion_xyzw: q,
        euler_deg: euler,
        raw: raw.clone(),
        fresh: true,
    });
    Ok(())
}

fn emit(shared: &SharedRef, sender: &mut Option<Sender>) -> Result<()> {
    let now = Instant::now();
    let (pose, source_id, age_ns) = {
        let s = lock(shared);
        let pose = s.pose.clone().map(|mut p| {
            p.fresh = fresh(&s, now);
            p
        });
        (
            pose,
            s.descriptor.source_id.clone(),
            s.last_pose.map_or(0, |at| {
                now.duration_since(at).as_nanos().min(i64::MAX as u128) as u64
            }),
        )
    };
    if let Some(sender) = sender {
        let previous_errors = sender.send_errors();
        let sent = if let Some(pose) = pose {
            sender.send_if_due(&pose, &source_id, age_ns, now)?
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
                    let mut angles = euler_deg;
                    match pattern {
                        Pattern::Fixed => {},
                        Pattern::Yaw => angles[0] += 45.0*(t*std::f64::consts::TAU*0.25).sin(),
                        Pattern::Combined => {
                            angles[0] += 45.0*(t*1.5).sin(); angles[1] += 20.0*(t*1.1).sin(); angles[2] += 15.0*(t*0.9).sin();
                        },
                        Pattern::Wrap => angles[0] = (170.0 + t*20.0 + 180.0).rem_euclid(360.0)-180.0,
                    }
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
                    publish(&shared,pose::from_euler(angles)?,&RawData::default(),start,now,sample_time)?;
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
    let mut last_bytes = start;
    let mut last_delivery: Option<Instant> = None;
    let mut next_link_check = start + Duration::from_secs(1);
    lock(shared).status.ble_link = connection.link_status();
    let mut parser = Parser::default();
    let mut raw = RawData::default();
    let mut sample_clock = SampleClock::default();
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
        let request_deadline = (config.pose_input == PoseInput::Quaternion)
            .then(|| outstanding.map_or(next_request, |sent| sent + Duration::from_millis(250)));
        tokio::select! {
            _ = cancel.changed() => return Err(Error::Cancelled),
            bytes = connection.read() => {
                let bytes = bytes?;
                last_bytes = Instant::now();
                let ns = last_bytes.duration_since(start).as_nanos().min(i64::MAX as u128) as u64;
                let old_discarded = parser.discarded_bytes;
                let old_invalid = parser.invalid_frames;
                let frames = parser.push(&bytes);
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
                for frame in frames {
                    lock(shared).status.frames_received += 1;
                    let mut sample_time_ms = None;
                    let q = match frame {
                        Frame::Motion { acceleration_g,angular_velocity_dps,euler_xyz_deg } => {
                            raw.acceleration_g=Some(acceleration_g);raw.angular_velocity_dps=Some(angular_velocity_dps);
                            raw.euler_xyz_deg=Some(euler_xyz_deg);raw.motion_received_ns=Some(ns);
                            if config.pose_input==PoseInput::Euler { Some(mounting.from_sensor_euler(euler_xyz_deg)) } else { None }
                        },
                        Frame::Stream { sample_time_ms: time, angular_velocity_dps, euler_xyz_deg, quaternion_wxyz } => {
                            // Partial stream groups must not retain old fields under a new host timestamp.
                            raw.acceleration_g = None;
                            raw.angular_velocity_dps = angular_velocity_dps;
                            raw.euler_xyz_deg = euler_xyz_deg;
                            raw.motion_received_ns = (angular_velocity_dps.is_some() || euler_xyz_deg.is_some()).then_some(ns);
                            if let Some(q) = quaternion_wxyz {
                                raw.quaternion_wxyz = Some(q);
                                raw.quaternion_received_ns = Some(ns);
                            }
                            sample_time_ms = time;
                            match config.pose_input {
                                PoseInput::Euler => euler_xyz_deg.map(|e| mounting.from_sensor_euler(e)),
                                PoseInput::StreamQuaternion => quaternion_wxyz.map(|[w,x,y,z]| mounting.from_sensor_quaternion([x,y,z,w])),
                                PoseInput::Quaternion => None,
                            }
                        },
                        Frame::Registers { address:0x51,values } => {
                            outstanding=None;
                            let q = [values[0],values[1],values[2],values[3]].map(|v| v as f64/32768.0);
                            raw.quaternion_wxyz=Some(q);raw.quaternion_received_ns=Some(ns);
                            if config.pose_input==PoseInput::Quaternion { Some(mounting.from_sensor_quaternion([q[1],q[2],q[3],q[0]])) } else { None }
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
                                if publish(shared,q,&raw,start,last_bytes,sample_time).is_err() {
                                    lock(shared).status.invalid_poses += 1;
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
                emit(shared,sender)?;
            },
            _ = wait_for_deadline(output_deadline) => emit(shared,sender)?,
            _ = wait_for_deadline(request_deadline) => {
                transport::cancel_after(cancel,Duration::from_secs(2),connection.write(&protocol::read_register(0x51))).await?;
                let sent = Instant::now();
                outstanding=Some(sent);
                next_request=sent+Duration::from_millis(20);
            },
            _ = ticks.tick() => {
                let now = Instant::now();
                if now >= next_link_check {
                    let link_status = connection.link_status();
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
                if start.elapsed()>Duration::from_secs(10) && lock(shared).status.frames_received==0 {
                    return Err(Error::Protocol("no supported WIT frames; check output format, model, firmware and baud rate".into()));
                }
            },
        }
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
    async fn cancellation_after_write_invalidates_reference_without_repeating_write() {
        let (mut device, host) = SerialStream::pair().unwrap();
        let mut connection = Connection::Usb(host);
        let shared = Arc::new(Mutex::new(Shared::default()));
        lock(&shared).operation = Some(OperationStatus::new("rate".into()));
        lock(&shared).descriptor.device.valid = true;
        let observed = shared.clone();
        let (tx, mut cancel) = watch::channel(false);
        let worker = tokio::spawn(async move {
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
        assert_eq!(bytes, protocol::UNLOCK);
        device.read_exact(&mut bytes).await.unwrap();
        assert_eq!(bytes, protocol::write_register(3, 9));
        tx.send(true).unwrap();
        assert!(matches!(worker.await.unwrap(), Err(Error::Cancelled)));
        let s = lock(&observed);
        assert!(!s.descriptor.device.valid);
        assert_eq!(s.descriptor.reference_epoch, 2);
        assert_eq!(s.descriptor.reference_reason, "device_control_uncertain");
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
        // Stream quaternion input is passive; it must not issue register requests.
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
        // A timestamp in a preceding stream frame must not be cached onto a register reply.
        device
            .write_all(&[0x55, 0x81, 15, 1, 1, 0, 0, 0, 5, 0, 0, 0, 0, 0, 0, 0])
            .await
            .unwrap();
        device.write_all(&response).await.unwrap();
        tokio::time::timeout(Duration::from_millis(200), device.read_exact(&mut command))
            .await
            .unwrap()
            .unwrap();
        assert_eq!(lock(&observed).status.pose_count, 1);
        assert!(lock(&observed).pose.as_ref().unwrap().sample_time.is_none());
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
            assert_eq!(sent[0], protocol::UNLOCK);
            assert_eq!(sent[1], protocol::write_register(address, value));
            if address != 0 {
                assert!(sent.iter().all(|c| c[2] != 0), "implicit SAVE");
            }
            server.abort();
            let _ = server.await;
        }
    }
}
