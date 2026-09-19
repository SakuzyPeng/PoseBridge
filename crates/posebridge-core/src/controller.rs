use crate::model::*;
use crate::osc::Sender;
use crate::pose;
use crate::protocol::{self, Frame, Parser};
use crate::transport::{self, Connection};
use futures_util::FutureExt;
use std::future::Future;
use std::panic::AssertUnwindSafe;
use std::sync::{
    Arc, Mutex, MutexGuard,
    atomic::{AtomicU64, Ordering},
    mpsc,
};
use std::time::{Duration, Instant};
use tokio::runtime::Runtime;
use tokio::sync::watch;

pub const STALE_AFTER: Duration = Duration::from_millis(500);
static NEXT_SESSION: AtomicU64 = AtomicU64::new(1);

#[derive(Default)]
struct Shared {
    status: StatusSnapshot,
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
        self.config = config;
        Ok(())
    }

    pub fn config(&self) -> &Config {
        &self.config
    }

    pub fn start(&mut self) -> Result<()> {
        self.config.validate()?;
        self.ensure_idle()?;
        let config = self.config.clone();
        *lock(&self.shared) = Shared::default();
        lock(&self.shared).status.state = ConnectionState::Connecting;
        self.launch(move |shared, cancel| run(config, shared, cancel));
        Ok(())
    }

    pub fn scan_start(&mut self, kind: TransportKind, seconds: u32) -> Result<()> {
        if !(1..=60).contains(&seconds) {
            return Err(Error::Invalid("scan duration must be 1..60 seconds".into()));
        }
        self.ensure_idle()?;
        *lock(&self.shared) = Shared::default();
        lock(&self.shared).status.state = ConnectionState::Scanning;
        self.launch(move |shared, mut cancel| async move {
            let devices =
                transport::scan(kind, Duration::from_secs(seconds as u64), &mut cancel).await?;
            lock(&shared).devices = devices;
            Ok(())
        });
        Ok(())
    }

    pub fn configure_device(&mut self, command: DeviceCommand) -> Result<()> {
        self.config.validate()?;
        protocol::command_register(&command)?;
        if matches!(self.config.source, Source::Simulate { .. }) {
            return Err(Error::Invalid("simulator has no device registers".into()));
        }
        self.ensure_idle()?;
        let source = self.config.source.clone();
        *lock(&self.shared) = Shared::default();
        lock(&self.shared).status.state = ConnectionState::Configuring;
        self.launch(move |shared, mut cancel| async move {
            let mut connection = Connection::open(&source, &mut cancel).await?;
            let result = apply_command(&mut connection, &command, &mut cancel).await;
            connection.close().await;
            match result {
                Ok(report) => {
                    lock(&shared).status.configuration_report = Some(report);
                    Ok(())
                }
                Err(e) => Err(e),
            }
        });
        Ok(())
    }

    fn launch<F, Fut>(&mut self, work: F)
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
                let result = AssertUnwindSafe(work(shared.clone(), rx))
                    .catch_unwind()
                    .await;
                let mut s = lock(&shared);
                match result {
                    Ok(Ok(())) => s.status.state = ConnectionState::Complete,
                    Ok(Err(Error::Cancelled)) => s.status.state = ConnectionState::Stopped,
                    Ok(Err(e)) => {
                        s.status.state = ConnectionState::Failed;
                        s.status.last_error = Some(e.to_string());
                    }
                    Err(_) => {
                        s.status.state = ConnectionState::Failed;
                        s.status.last_error = Some("internal worker panic".into());
                    }
                }
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

    pub fn status(&self) -> StatusSnapshot {
        let s = lock(&self.shared);
        let mut status = s.status.clone();
        if status.state == ConnectionState::Active && !fresh(&s, Instant::now()) {
            status.state = ConnectionState::Stale;
        }
        status
    }

    pub fn latest_pose(&self) -> Option<PoseSnapshot> {
        let s = lock(&self.shared);
        s.pose.clone().map(|mut p| {
            p.fresh = fresh(&s, Instant::now());
            p
        })
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

fn begin_session(shared: &SharedRef) {
    let mut s = lock(shared);
    let reconnect_count = s.status.reconnect_count;
    *s = Shared::default();
    s.status.session_id = NEXT_SESSION.fetch_add(1, Ordering::Relaxed);
    s.status.reconnect_count = reconnect_count;
    s.status.state = ConnectionState::Connecting;
}

fn publish(shared: &SharedRef, q: pose::Quat, raw: &RawData, start: Instant) -> Result<()> {
    let q = pose::normalize(q)?;
    let euler = pose::to_euler(q)?;
    let now = Instant::now();
    let mut s = lock(shared);
    if let Some(last) = s.last_pose {
        let gap = now.duration_since(last).as_secs_f64() * 1000.0;
        if s.status.interval_min_ms == 0.0 || gap < s.status.interval_min_ms {
            s.status.interval_min_ms = gap;
        }
        s.status.interval_max_ms = s.status.interval_max_ms.max(gap);
    }
    let first = *s.first_pose.get_or_insert(now);
    s.last_pose = Some(now);
    s.status.pose_count += 1;
    let elapsed = now.duration_since(first).as_secs_f64();
    s.status.actual_rate_hz = if elapsed > 0.0 {
        (s.status.pose_count - 1) as f64 / elapsed
    } else {
        0.0
    };
    s.status.state = ConnectionState::Active;
    s.pose = Some(PoseSnapshot {
        session_id: s.status.session_id,
        sequence: s.status.pose_count,
        received_ns: now.duration_since(start).as_nanos().min(u64::MAX as u128) as u64,
        quaternion_xyzw: q,
        euler_deg: euler,
        raw: raw.clone(),
        fresh: true,
    });
    Ok(())
}

fn emit(shared: &SharedRef, sender: &mut Option<Sender>) -> Result<()> {
    let now = Instant::now();
    let pose = {
        let mut s = lock(shared);
        if s.status.state == ConnectionState::Active && !fresh(&s, now) {
            s.status.state = ConnectionState::Stale;
        }
        s.pose.clone().map(|mut p| {
            p.fresh = fresh(&s, now);
            p
        })
    };
    if let (Some(sender), Some(pose)) = (sender, pose)
        && sender.send_if_due(&pose, now)?
    {
        lock(shared).status.osc_sent += 1;
    }
    Ok(())
}

async fn run(config: Config, shared: SharedRef, mut cancel: watch::Receiver<bool>) -> Result<()> {
    let mut sender = config.osc.clone().map(Sender::new).transpose()?;
    if let Source::Simulate {
        pattern,
        euler_deg,
        rate_hz,
    } = config.source
    {
        begin_session(&shared);
        let start = Instant::now();
        let mut sampling = tokio::time::interval(Duration::from_secs_f64(1.0 / rate_hz as f64));
        sampling.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
        let mut output = tokio::time::interval(Duration::from_millis(5));
        output.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
        loop {
            tokio::select! {
                _ = cancel.changed() => return Err(Error::Cancelled),
                _ = sampling.tick() => {
                    let t = start.elapsed().as_secs_f64();
                    let mut angles = euler_deg;
                    match pattern {
                        Pattern::Fixed => {},
                        Pattern::Yaw => angles[0] += 45.0*(t*std::f64::consts::TAU*0.25).sin(),
                        Pattern::Combined => {
                            angles[0] += 45.0*(t*1.5).sin(); angles[1] += 20.0*(t*1.1).sin(); angles[2] += 15.0*(t*0.9).sin();
                        },
                        Pattern::Wrap => angles[0] = (170.0 + t*20.0 + 180.0).rem_euclid(360.0)-180.0,
                    }
                    publish(&shared,pose::from_euler(angles)?,&RawData::default(),start)?;
                },
                _ = output.tick() => emit(&shared,&mut sender)?,
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
                let result =
                    pump(&config, &shared, &mut connection, &mut sender, &mut cancel).await;
                connection.close().await;
                if lock(&shared).status.pose_count > 0 {
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
    let mut parser = Parser::default();
    let mut raw = RawData::default();
    let mounting = config
        .mounting
        .ok_or_else(|| Error::Invalid("mounting required".into()))?;
    let mut next_request = start;
    let mut outstanding: Option<Instant> = None;
    let mut ticks = tokio::time::interval(Duration::from_millis(5));
    ticks.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
    loop {
        tokio::select! {
            _ = cancel.changed() => return Err(Error::Cancelled),
            bytes = connection.read() => {
                let bytes = bytes?;
                last_bytes = Instant::now();
                let ns = start.elapsed().as_nanos().min(u64::MAX as u128) as u64;
                lock(shared).status.bytes_received += bytes.len() as u64;
                for frame in parser.push(&bytes) {
                    lock(shared).status.frames_received += 1;
                    let q = match frame {
                        Frame::Motion { acceleration_g,angular_velocity_dps,euler_xyz_deg } => {
                            raw.acceleration_g=Some(acceleration_g);raw.angular_velocity_dps=Some(angular_velocity_dps);
                            raw.euler_xyz_deg=Some(euler_xyz_deg);raw.motion_received_ns=Some(ns);
                            if config.pose_input==PoseInput::Euler { Some(mounting.from_sensor_euler(euler_xyz_deg)) } else { None }
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
                        match q.and_then(|q|publish(shared,q,&raw,start)) {
                            Ok(()) => {},
                            Err(_) => lock(shared).status.invalid_poses+=1,
                        }
                    }
                }
                lock(shared).status.discarded_bytes=parser.discarded_bytes;
            },
            _ = ticks.tick() => {
                emit(shared,sender)?;
                let now = Instant::now();
                if now.duration_since(start) >= STALE_AFTER {
                    let mut state = lock(shared);
                    if state.pose.is_none() {
                        state.status.state = ConnectionState::Stale;
                    }
                }
                if now.duration_since(last_bytes)>Duration::from_secs(10) { return Err(Error::Timeout("no device bytes for 10 seconds".into())); }
                if start.elapsed()>Duration::from_secs(10) && lock(shared).status.frames_received==0 {
                    return Err(Error::Protocol("no WIT 20-byte frames; check model, firmware and baud rate".into()));
                }
                if config.pose_input==PoseInput::Quaternion && now>=next_request
                    && outstanding.is_none_or(|t|now.duration_since(t)>=Duration::from_millis(250)) {
                    transport::cancel_after(cancel,Duration::from_secs(2),connection.write(&protocol::read_register(0x51))).await?;
                    outstanding=Some(now);next_request=now+Duration::from_millis(20);
                }
            },
        }
    }
}

async fn read_back(
    connection: &mut Connection,
    address: u16,
    cancel: &mut watch::Receiver<bool>,
) -> Result<u16> {
    transport::cancel_after(cancel, Duration::from_secs(3), async {
        connection.write(&protocol::read_register(address)).await?;
        let mut parser = Parser::default();
        loop {
            for frame in parser.push(&connection.read().await?) {
                if let Frame::Registers {
                    address: base,
                    values,
                } = frame
                    && address >= base
                    && address - base < 8
                {
                    return Ok(values[(address - base) as usize] as u16);
                }
            }
        }
    })
    .await
}

async fn apply_command(
    connection: &mut Connection,
    command: &DeviceCommand,
    cancel: &mut watch::Receiver<bool>,
) -> Result<String> {
    let (address, value) = protocol::command_register(command)?;
    transport::cancel_after(
        cancel,
        Duration::from_secs(2),
        connection.write(&protocol::UNLOCK),
    )
    .await?;
    transport::cancel_after(cancel, Duration::from_secs(1), async {
        tokio::time::sleep(Duration::from_millis(100)).await;
        Ok(())
    })
    .await?;
    transport::cancel_after(
        cancel,
        Duration::from_secs(2),
        connection.write(&protocol::write_register(address, value)),
    )
    .await?;
    let observed = read_back(connection, address as u16, cancel).await?;
    if matches!(command, DeviceCommand::AccelCalibrate) {
        // This action auto-clears CALSW. A quick zero readback cannot prove that calibration started.
        if observed != 1 {
            return Err(Error::Protocol(format!(
                "calibration command sent; start not observed (CALSW={observed}), completion unverified"
            )));
        }
        let start = Instant::now();
        while start.elapsed() < Duration::from_secs(8) {
            transport::cancel_after(cancel, Duration::from_secs(1), async {
                tokio::time::sleep(Duration::from_millis(100)).await;
                Ok(())
            })
            .await?;
            if read_back(connection, address as u16, cancel).await? == 0 {
                return Ok(
                    "CALSW start and completion observed; accuracy requires a separate measurement"
                        .into(),
                );
            }
        }
        return Err(Error::Timeout(
            "calibration completion was not observed".into(),
        ));
    }
    if observed != value {
        return Err(Error::Protocol(format!(
            "register 0x{address:02x}: requested {value}, read back {observed}"
        )));
    }
    Ok(if matches!(command, DeviceCommand::Save) {
        "save command sent; SAVE register readback matched; persistence requires a power-cycle check".into()
    } else {
        format!(
            "register 0x{address:02x} readback verified: {observed}; output rate/accuracy still require measurement"
        )
    })
}

#[cfg(all(test, unix))]
mod tests {
    use super::*;
    use crate::pose::Mounting;
    use std::net::UdpSocket;
    use tokio::io::{AsyncReadExt, AsyncWriteExt};
    use tokio_serial::SerialStream;

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
            assert_eq!(message.addr, "/posebridge/v1/euler");
            let expected = [
                -5239.0 / 32768.0 * 180.0,
                97.0 / 32768.0 * 180.0,
                139.0 / 32768.0 * 180.0,
            ];
            for (arg, value) in message.args.iter().zip(expected) {
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
            let result = apply_command(&mut connection, &command, &mut cancel).await;
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
