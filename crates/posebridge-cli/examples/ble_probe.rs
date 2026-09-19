//! Experimental BLE delivery probe, not an OSC bridge. Never writes configuration.
//! Optional polling sends only documented WIT read-register commands.
use btleplug::api::{Central, CharPropFlags, Manager as _, Peripheral as _, ScanFilter, WriteType};
use btleplug::platform::{Manager, Peripheral};
use clap::{Parser, ValueEnum};
use futures_util::StreamExt;
use posebridge_core::protocol::read_register;
use serde_json::json;
use std::collections::{BTreeMap, VecDeque};
use std::path::PathBuf;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};
use uuid::Uuid;

type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;
#[derive(Clone, Copy, ValueEnum)]
enum WriteMode {
    Response,
    Command,
}

#[derive(Parser)]
struct Args {
    #[arg(long)]
    device: String,
    #[arg(long, default_value_t = 8)]
    seconds: u64,
    #[arg(long, default_value_t = 2)]
    warmup: u64,
    /// Whitelisted read address: 0x03, 0x0e, 0x1f, 0x2e, 0x3d or 0x51.
    #[arg(long, value_parser = parse_register)]
    register: Option<u16>,
    #[arg(long, default_value_t = 50, value_parser = clap::value_parser!(u32).range(1..=200))]
    poll_hz: u32,
    #[arg(long, default_value_t = 1, value_parser = clap::value_parser!(u32).range(1..=4))]
    window: u32,
    #[arg(long, value_enum, default_value = "response")]
    write_mode: WriteMode,
    #[arg(long)]
    raw_output: Option<PathBuf>,
}

fn parse_register(s: &str) -> std::result::Result<u16, String> {
    let n = if let Some(hex) = s.strip_prefix("0x") {
        u16::from_str_radix(hex, 16)
    } else {
        s.parse()
    }
    .map_err(|e| format!("invalid register: {e}"))?;
    if [3, 0x0e, 0x1f, 0x2e, 0x3d, 0x51].contains(&n) {
        Ok(n)
    } else {
        Err("only documented read-only probes are accepted".into())
    }
}

fn frame_len(flag: u8) -> Option<usize> {
    if flag == 0x71 {
        return Some(20);
    }
    if flag == 0 || flag & 0x10 != 0 {
        return None;
    }
    Some(
        2 + [
            (0x80, 8),
            (0x40, 6),
            (0x20, 6),
            (8, 6),
            (4, 8),
            (2, 12),
            (1, 6),
        ]
        .iter()
        .filter(|(mask, _)| flag & mask != 0)
        .map(|(_, size)| size)
        .sum::<usize>(),
    )
}

#[derive(Default)]
struct Reads {
    pending: VecDeque<Instant>,
    sent: u64,
    timeouts: u64,
    failures: Vec<String>,
}

fn gaps(times: &[f64]) -> serde_json::Value {
    let mut gaps: Vec<_> = times.windows(2).map(|w| (w[1] - w[0]) * 1000.).collect();
    gaps.sort_by(f64::total_cmp);
    if gaps.is_empty() {
        return json!(null);
    }
    json!({"p50_ms":gaps[gaps.len()/2],"p95_ms":gaps[(gaps.len()*95/100).min(gaps.len()-1)],"max_ms":gaps.last()})
}

fn rate(times: &[f64]) -> f64 {
    if times.len() < 2 || times.last() == times.first() {
        return 0.;
    }
    (times.len() - 1) as f64 / (times.last().unwrap() - times[0])
}

async fn observe(peripheral: &Peripheral, args: &Args) -> Result<()> {
    let characteristics = peripheral.characteristics();
    let uuid = |short: u128| Uuid::from_u128((short << 96) | 0x00001000800000805f9a34fb);
    let notify = characteristics
        .iter()
        .find(|c| c.uuid == uuid(0xffe4) && c.service_uuid == uuid(0xffe5))
        .ok_or("FFE5/FFE4 missing")?;
    let writer = characteristics
        .iter()
        .find(|c| c.uuid == uuid(0xffe9) && c.service_uuid == uuid(0xffe5))
        .cloned();
    let inventory: Vec<_> = characteristics.iter().map(|c| json!({"service":c.service_uuid.to_string(),"uuid":c.uuid.to_string(),"properties":format!("{:?}",c.properties)})).collect();
    let mut notifications = peripheral.notifications().await?;
    peripheral.subscribe(notify).await?;
    let start = Instant::now();
    let start_unix_ns = SystemTime::now().duration_since(UNIX_EPOCH)?.as_nanos();
    let reads = Arc::new(Mutex::new(Reads::default()));
    let worker = if let Some(register) = args.register {
        let writer = writer.ok_or("FFE9 missing")?;
        let mode = match args.write_mode {
            WriteMode::Response if writer.properties.contains(CharPropFlags::WRITE) => {
                WriteType::WithResponse
            }
            WriteMode::Command
                if writer
                    .properties
                    .contains(CharPropFlags::WRITE_WITHOUT_RESPONSE) =>
            {
                WriteType::WithoutResponse
            }
            _ => return Err("requested write mode not supported".into()),
        };
        let peripheral = peripheral.clone();
        let reads = reads.clone();
        let window = args.window as usize;
        let hz = args.poll_hz;
        Some(tokio::spawn(async move {
            let mut tick = tokio::time::interval(Duration::from_secs_f64(1. / hz as f64));
            tick.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
            loop {
                tick.tick().await;
                let now = Instant::now();
                {
                    let mut s = reads.lock().unwrap();
                    while s
                        .pending
                        .front()
                        .is_some_and(|t| now.duration_since(*t) >= Duration::from_millis(300))
                    {
                        s.pending.pop_front();
                        s.timeouts += 1;
                    }
                    if s.pending.len() >= window {
                        continue;
                    }
                    s.pending.push_back(now);
                    s.sent += 1;
                }
                match tokio::time::timeout(
                    Duration::from_secs(2),
                    peripheral.write(&writer, &read_register(register), mode),
                )
                .await
                {
                    Ok(Ok(())) => {}
                    failure => {
                        reads.lock().unwrap().failures.push(format!("{failure:?}"));
                        return;
                    }
                }
            }
        }))
    } else {
        None
    };
    let end = start + Duration::from_secs(args.seconds + args.warmup);
    let mut buffer = VecDeque::<u8>::new();
    let mut discarded = 0;
    let mut logs = Vec::new();
    let mut deliveries = Vec::new();
    let mut samples = Vec::new();
    let mut replies = Vec::new();
    let mut sizes = BTreeMap::<usize, u64>::new();
    let mut flags = BTreeMap::<String, u64>::new();
    let mut complete_pose_deliveries = 0u64;
    let mut timestamp_values = Vec::<u64>::new();
    let mut previous_pose: Option<Vec<u8>> = None;
    let mut changed_poses = 0u64;
    let mut pose_frames = 0u64;
    let mut invalid_timestamps = 0u64;
    let mut invalid_quaternions = 0u64;
    let mut quaternion_norm_min: Option<f64> = None;
    let mut quaternion_norm_max: Option<f64> = None;
    let result:Result<()>=async {
        loop {
            let value=tokio::select! {
                _=tokio::time::sleep_until(end.into()) => break,
                next=notifications.next() => next.ok_or("notification stream ended")?,
            };
            if value.uuid!=notify.uuid {continue;}
            let now=Instant::now();if now>=end {break;}
            let t=now.duration_since(start).as_secs_f64();let measured=t>=args.warmup as f64;
            if args.raw_output.is_some(){logs.push(json!({"t":t,"hex":value.value.iter().map(|b|format!("{b:02x}")).collect::<String>()}));}
            if measured {deliveries.push(t);*sizes.entry(value.value.len()).or_default()+=1;}
            buffer.extend(&value.value);let mut has_pose=false;
            while buffer.len()>=2 {
                if buffer[0]!=0x55 || frame_len(buffer[1]).is_none() {buffer.pop_front();discarded+=1;continue;}
                let len=frame_len(buffer[1]).unwrap();if buffer.len()<len {break;}
                let frame:Vec<_>=buffer.drain(..len).collect();let flag=frame[1];
                if measured {*flags.entry(format!("0x{flag:02x}")).or_default()+=1;}
                if flag==0x71 {
                    let address=u16::from_le_bytes([frame[2],frame[3]]);
                    if Some(address)==args.register {reads.lock().unwrap().pending.pop_front();if measured {replies.push(t);}}
                    if measured && [0x3d,0x51].contains(&address) {has_pose=true;}
                } else {
                    if flag&0x80!=0 && (!(1..=12).contains(&frame[3]) || !(1..=31).contains(&frame[4]) || frame[5]>23 || frame[6]>59 || frame[7]>59 || u16::from_le_bytes([frame[8],frame[9]])>=1000) {
                        if measured {invalid_timestamps+=1;}continue;
                    }
                    if flag&4!=0 {
                        let offset=2+[(0x80,8),(0x40,6),(0x20,6),(8,6)].iter().filter(|(bit,_)|flag&bit!=0).map(|(_,size)|size).sum::<usize>();
                        let norm=(0..4).map(|i| (i16::from_le_bytes([frame[offset+i*2],frame[offset+i*2+1]]) as f64/32768.0).powi(2)).sum::<f64>().sqrt();
                        if measured {
                            quaternion_norm_min=Some(quaternion_norm_min.map_or(norm,|n|n.min(norm)));
                            quaternion_norm_max=Some(quaternion_norm_max.map_or(norm,|n|n.max(norm)));
                        }
                        if !(0.95..=1.05).contains(&norm) {if measured {invalid_quaternions+=1;}continue;}
                    }
                    if measured {samples.push(t);}
                    if flag & 0x80 != 0 && measured {
                        timestamp_values.push(((frame[5] as u64*60+frame[6] as u64)*60+frame[7] as u64)*1000+u16::from_le_bytes([frame[8],frame[9]]) as u64);
                    }
                    if flag & 5 != 0 {
                        if measured {has_pose=true;pose_frames+=1;}
                        // Euler occupies the last six bytes in all documented stream layouts.
                        if flag&1!=0 {
                            let pose=frame[len-6..].to_vec();
                            if measured && previous_pose.as_ref().is_some_and(|p|*p!=pose) {changed_poses+=1;}
                            previous_pose=Some(pose);
                        }
                    }
                }
            }
            if measured && has_pose {complete_pose_deliveries+=1;}
        }
        Ok(())
    }.await;
    if let Some(worker) = worker {
        worker.abort();
        let _ = worker.await;
    }
    let _ = peripheral.unsubscribe(notify).await;
    result?;
    if let Some(path) = &args.raw_output {
        std::fs::write(
            path,
            logs.iter().map(|v| format!("{v}\n")).collect::<String>(),
        )?;
    }
    let reads = reads.lock().unwrap();
    let timestamp_deltas: Vec<_> = timestamp_values
        .windows(2)
        .map(|v| v[1] as i64 - v[0] as i64)
        .collect();
    let mut timestamp_histogram = BTreeMap::new();
    for delta in timestamp_deltas {
        *timestamp_histogram.entry(delta.to_string()).or_insert(0u64) += 1;
    }
    println!(
        "{}",
        json!({"seconds":args.seconds,"warmup_seconds":args.warmup,"start_unix_ns":start_unix_ns.to_string(),"notifications":deliveries.len(),"notification_hz":rate(&deliveries),"notification_gaps":gaps(&deliveries),"notification_sizes":sizes,"stream_frames":samples.len(),"stream_hz":samples.len() as f64/args.seconds as f64,"flags":flags,"pose_frames":pose_frames,"complete_pose_notifications":complete_pose_deliveries,"pose_notification_hz":complete_pose_deliveries as f64/args.seconds as f64,"changed_angle_pairs":changed_poses,"register":args.register,"responses":replies.len(),"response_hz":rate(&replies),"requests_including_warmup":reads.sent,"request_timeouts":reads.timeouts,"write_errors":reads.failures,"discarded_bytes":discarded,"invalid_timestamps":invalid_timestamps,"invalid_quaternions":invalid_quaternions,"quaternion_norm_min":quaternion_norm_min,"quaternion_norm_max":quaternion_norm_max,"timestamp_delta_ms_histogram":timestamp_histogram,"characteristics":inventory})
    );
    Ok(())
}

#[tokio::main(flavor = "multi_thread", worker_threads = 2)]
async fn main() -> Result<()> {
    let args = Args::parse();
    if !(1..=60).contains(&args.seconds) || args.warmup > 10 {
        return Err("seconds must be 1..60, warmup 0..10".into());
    }
    let manager = Manager::new().await?;
    let adapter = manager
        .adapters()
        .await?
        .into_iter()
        .next()
        .ok_or("no BLE adapter")?;
    adapter.start_scan(ScanFilter::default()).await?;
    let found = tokio::time::timeout(Duration::from_secs(12), async {
        loop {
            for p in adapter.peripherals().await? {
                if p.id().to_string() == args.device {
                    return Ok::<_, btleplug::Error>(p);
                }
            }
            tokio::time::sleep(Duration::from_millis(100)).await;
        }
    })
    .await;
    let _ = adapter.stop_scan().await;
    let peripheral = found??;
    let result: Result<()> = async {
        tokio::time::timeout(Duration::from_secs(12), peripheral.connect()).await??;
        tokio::time::timeout(Duration::from_secs(12), peripheral.discover_services()).await??;
        observe(&peripheral, &args).await
    }
    .await;
    let _ = tokio::time::timeout(Duration::from_secs(3), peripheral.disconnect()).await;
    result
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn documented_variable_lengths_and_read_whitelist() {
        for (flag, len) in [
            (0x61, 20),
            (0xe1, 28),
            (0x69, 26),
            (0x6d, 34),
            (0x6f, 46),
            (1, 8),
            (4, 10),
            (0x81, 16),
            (0x71, 20),
        ] {
            assert_eq!(frame_len(flag), Some(len));
        }
        assert_eq!(frame_len(0x10), None);
        assert!(parse_register("0x00").is_err());
        assert!(parse_register("0x51").is_ok());
    }
}
