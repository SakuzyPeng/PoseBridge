//! Acquisition-only aggregate checker. Use tests/full_inertial.py for temporary
//! configuration and guaranteed attempted restoration on failure/interruption.
use posebridge_core::{Config, ConnectionState, Controller, MotionSample, pose};
use serde_json::json;
use std::time::{Duration, Instant};

fn norm(v: [f64; 3]) -> f64 {
    v.iter().map(|x| x * x).sum::<f64>().sqrt()
}
fn conjugate([x, y, z, w]: [f64; 4]) -> [f64; 4] {
    [-x, -y, -z, w]
}
fn rotation_vector(a: &MotionSample, b: &MotionSample) -> [f64; 3] {
    let mut delta = pose::multiply(conjugate(a.orientation_xyzw), b.orientation_xyzw);
    if delta[3] < 0. {
        delta = delta.map(|v| -v);
    }
    let n = norm([delta[0], delta[1], delta[2]]);
    let scale = if n > 1e-9 {
        2. * n.atan2(delta[3]) / n
    } else {
        2.
    };
    [delta[0] * scale, delta[1] * scale, delta[2] * scale]
}
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<_> = std::env::args().collect();
    if args.len() != 3 {
        return Err("usage: motion_probe '<Config JSON>' <capture seconds>".into());
    }
    let seconds: u32 = args[2].parse()?;
    if !(1..=600).contains(&seconds) {
        return Err("capture must be 1..600 seconds".into());
    }
    let mut config: Config = serde_json::from_str(&args[1])?;
    config.osc = None;
    let mut controller = Controller::new()?;
    controller.set_config(config)?;
    controller.start()?;
    let deadline = Instant::now() + Duration::from_secs(u64::from(seconds) + 30);
    let mut cursor = None;
    let mut first_delivery = None;
    let mut capture = None;
    let mut baseline = None;
    let mut previous: Option<MotionSample> = None;
    let mut samples = 0u64;
    let mut missing_fields = 0u64;
    let mut time_errors = 0u64;
    let mut overruns = 0u64;
    let mut direction_checks = 0u64;
    let mut direction_errors = 0u64;
    let mut max_residual = 0f64;
    let mut static_seconds = 0f64;
    let mut axis_degrees = [0f64; 3];
    loop {
        let batch = controller.motion_since(cursor);
        cursor = batch.cursor;
        let now = batch.queried_at;
        if !batch.samples.is_empty() {
            first_delivery.get_or_insert(now);
        }
        if capture.is_none()
            && first_delivery.is_some_and(|first| now - first >= Duration::from_secs(2))
        {
            capture = Some(now);
            baseline = Some(controller.snapshot().status);
            eprintln!(
                "CAPTURE_READY: hold still at least 5 seconds, then nod, shake and tilt through at least 20 degrees each; capture {seconds}s"
            );
        }
        if capture.is_some() {
            overruns += batch.history_overrun;
            if batch.reset {
                time_errors += 1;
            }
            for sample in batch.samples {
                samples += 1;
                if sample.profile != 0xe4
                    || sample.sample_time.is_none()
                    || sample.angular_velocity_rad_s.is_none()
                    || sample.acceleration_g.is_none()
                {
                    missing_fields += 1;
                }
                if let Some(old) = &previous {
                    match (
                        old.sample_time,
                        sample.sample_time,
                        old.angular_velocity_rad_s,
                        sample.angular_velocity_rad_s,
                    ) {
                        (Some(a), Some(b), Some(w0), Some(w1))
                            if b.time_ms > a.time_ms
                                && b.clock_epoch == a.clock_epoch
                                && old.cursor.session_id == sample.cursor.session_id =>
                        {
                            let dt = (b.time_ms - a.time_ms) as f64 / 1000.;
                            let observed = rotation_vector(old, &sample);
                            let integrated = std::array::from_fn(|i| (w0[i] + w1[i]) * 0.5 * dt);
                            let residual =
                                norm(std::array::from_fn(|i| observed[i] - integrated[i]))
                                    .to_degrees();
                            max_residual = max_residual.max(residual);
                            if norm(w1) < 1f64.to_radians()
                                && norm(observed) / dt < 1f64.to_radians()
                                && sample
                                    .acceleration_g
                                    .is_some_and(|a| (norm(a) - 1.).abs() < 0.1)
                            {
                                static_seconds += dt;
                            }
                            if norm(integrated) > 0.5f64.to_radians()
                                && norm(observed) > 0.5f64.to_radians()
                            {
                                direction_checks += 1;
                                let dot = observed
                                    .iter()
                                    .zip(integrated)
                                    .map(|(a, b)| a * b)
                                    .sum::<f64>();
                                if dot / (norm(observed) * norm(integrated)) < 0.8 {
                                    direction_errors += 1;
                                }
                                for i in 0..3 {
                                    if w1[i].abs() > 10f64.to_radians() {
                                        axis_degrees[i] += observed[i].abs().to_degrees();
                                    }
                                }
                            }
                        }
                        _ => time_errors += 1,
                    }
                }
                previous = Some(sample);
            }
        }
        if capture.is_some_and(|start| now - start >= Duration::from_secs(u64::from(seconds))) {
            break;
        }
        let snapshot = controller.snapshot();
        if snapshot.status.state == ConnectionState::Failed || now >= deadline {
            return Err(format!("acquisition failed or timed out: {:?}", snapshot.status).into());
        }
        std::thread::sleep(Duration::from_millis(5));
    }
    let elapsed = capture.unwrap().elapsed().as_secs_f64();
    let status = controller.snapshot().status;
    controller.stop()?;
    let base = baseline.unwrap();
    let rate = samples as f64 / elapsed;
    let parser_errors = status.invalid_frames - base.invalid_frames;
    let discarded_bytes = status.discarded_bytes - base.discarded_bytes;
    let invalid_poses = status.invalid_poses - base.invalid_poses;
    let duplicates = status.duplicate_sample_times - base.duplicate_sample_times;
    let clock_resets = status.clock_discontinuities - base.clock_discontinuities;
    let reconnects = status.reconnect_count - base.reconnect_count;
    let passed = seconds >= 60
        && (19.0..=21.0).contains(&rate)
        && missing_fields
            + time_errors
            + overruns
            + parser_errors
            + discarded_bytes
            + invalid_poses
            + duplicates
            + clock_resets
            + reconnects
            + direction_errors
            == 0
        && max_residual <= 5.
        && static_seconds >= 5.
        && direction_checks >= 20
        && axis_degrees.iter().all(|v| *v >= 20.);
    println!(
        "{}",
        json!({
            "passed":passed, "elapsed_seconds":elapsed, "samples":samples, "rate_hz":rate,
            "missing_fields":missing_fields, "time_errors":time_errors, "history_overruns":overruns,
            "invalid_frames":parser_errors, "discarded_bytes":discarded_bytes, "invalid_poses":invalid_poses,
            "duplicate_times":duplicates,"clock_resets":clock_resets,"reconnects":reconnects,
            "direction_checks":direction_checks,"direction_errors":direction_errors,
            "max_integration_residual_degrees":max_residual,"static_seconds":static_seconds,
            "axis_motion_degrees":axis_degrees
        })
    );
    Ok(())
}
