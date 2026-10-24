//! Physical X-right/Y-forward/Z-up attitude, without consumer processing.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Quaternion(pub [f64; 4]);
impl Default for Quaternion {
    fn default() -> Self {
        Self([1.0, 0.0, 0.0, 0.0])
    }
}
impl Quaternion {
    pub fn normalized(self) -> Self {
        let length = self.0.iter().map(|x| x * x).sum::<f64>().sqrt();
        if !length.is_finite() || length < 1.0e-12 {
            return Self::default();
        }
        Self(self.0.map(|x| x / length))
    }
    pub fn conjugate(self) -> Self {
        let [w, x, y, z] = self.0;
        Self([w, -x, -y, -z])
    }
    pub fn multiply(self, rhs: Self) -> Self {
        let [aw, ax, ay, az] = self.0;
        let [bw, bx, by, bz] = rhs.0;
        Self([
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        ])
    }
    #[cfg(test)]
    pub fn from_euler([yaw, pitch, roll]: [f32; 3]) -> Self {
        let (sy, cy) = (f64::from(yaw).to_radians() * 0.5).sin_cos();
        let (sp, cp) = (f64::from(pitch).to_radians() * 0.5).sin_cos();
        let (sr, cr) = (f64::from(roll).to_radians() * 0.5).sin_cos();
        Self([cy, 0.0, 0.0, sy])
            .multiply(Self([cp, sp, 0.0, 0.0]))
            .multiply(Self([cr, 0.0, sr, 0.0]))
            .normalized()
    }
    #[allow(
        clippy::cast_possible_truncation,
        reason = "unit quaternion angles are bounded degrees"
    )]
    #[cfg(test)]
    pub fn euler(self) -> [f32; 3] {
        let [w, x, y, z] = self.normalized().0;
        let sin_pitch = (2.0 * (y * z + w * x)).clamp(-1.0, 1.0);
        let (yaw, roll) = if sin_pitch.abs() >= 1.0 - 1e-12 {
            // ZXY poles couple yaw and roll; choose roll=0 and preserve rotation.
            (
                (2.0 * (x * y + w * z)).atan2(1.0 - 2.0 * (y * y + z * z)),
                0.0,
            )
        } else {
            (
                (-(2.0 * (x * y - w * z))).atan2(1.0 - 2.0 * (x * x + z * z)),
                (-(2.0 * (x * z - w * y))).atan2(1.0 - 2.0 * (x * x + y * y)),
            )
        };
        [
            yaw.to_degrees() as f32,
            sin_pitch.asin().to_degrees() as f32,
            roll.to_degrees() as f32,
        ]
    }
}

use eframe::egui;
use posebridge_core as pb;
use std::time::{Duration, Instant};

#[derive(Clone, Debug)]
pub struct Sample {
    pub instance: u64,
    pub session: u64,
    pub reference: u64,
    pub sequence: u64,
    pub physical: Quaternion,
    pub angles: [f64; 3],
    pub fresh: bool,
    pub age: Duration,
    pub queried_at: Instant,
    pub gyro: Option<[f64; 3]>,
    pub acceleration: Option<[f64; 3]>,
    pub sample_time: Option<pb::SampleTime>,
    pub orientation_source: pb::OrientationSource,
}
impl Sample {
    pub fn from_motion(p: &pb::MotionSample, queried_at: Instant) -> Self {
        let [x, y, z, w] = p.orientation_xyzw;
        Self {
            instance: p.cursor.instance_id,
            session: p.cursor.session_id,
            reference: p.reference_epoch,
            sequence: p.cursor.sequence,
            physical: Quaternion([w, x, y, z]),
            angles: p.euler_deg,
            fresh: p.fresh,
            age: Duration::from_nanos(p.age_ns),
            queried_at,
            gyro: p.angular_velocity_rad_s,
            acceleration: p.acceleration_g,
            sample_time: p.sample_time,
            orientation_source: p.orientation_source,
        }
    }
    pub fn age_at(&self, now: Instant) -> Duration {
        self.age
            .saturating_add(now.saturating_duration_since(self.queried_at))
    }
    pub fn valid_at(&self, now: Instant) -> bool {
        self.fresh && self.age_at(now) < Duration::from_millis(500)
    }
}
#[derive(Clone, Copy, Default, Debug, PartialEq, Eq)]
pub enum TrackingStatus {
    #[default]
    Waiting,
    Active,
    Stale,
}
impl TrackingStatus {
    pub fn label(self) -> &'static str {
        match self {
            Self::Waiting => "Connect a sensor to begin",
            Self::Active => "Receiving sensor orientation",
            Self::Stale => "Stale · last orientation shown",
        }
    }
}
#[derive(Clone, Default)]
pub struct TrackingSnapshot {
    pub pose: Quaternion,
    pub status: TrackingStatus,
    pub measurement: Option<Quaternion>,
}
impl TrackingSnapshot {
    pub fn from_sample(sample: Option<&Sample>, now: Instant) -> Self {
        let Some(p) = sample else {
            return Self::default();
        };
        let fresh = p.valid_at(now);
        Self {
            pose: p.physical,
            status: if fresh {
                TrackingStatus::Active
            } else {
                TrackingStatus::Stale
            },
            measurement: fresh.then_some(p.physical),
        }
    }
}

pub fn draw(ui: &mut egui::Ui, head: &TrackingSnapshot, sample: Option<&Sample>) {
    ui.colored_label(
        if head.status == TrackingStatus::Active {
            crate::theme::SUCCESS
        } else {
            crate::theme::MUTED
        },
        head.status.label(),
    );
    if let Some(p) = sample {
        let [yaw, pitch, roll] = p.angles;
        ui.horizontal_wrapped(|ui| {
            for text in [
                format!("Yaw {yaw:.1}°"),
                format!("Pitch {pitch:.1}°"),
                format!("Roll {roll:.1}°"),
            ] {
                ui.label(text);
            }
        });
        ui.small(format!(
            "Reception age: {:.1} ms · sample {}",
            p.age_at(Instant::now()).as_secs_f64() * 1000.,
            p.sequence
        ));
    } else {
        ui.label("Yaw —   Pitch —   Roll —");
    }
    let size = egui::vec2(ui.available_width().min(400.), 220.);
    let (rect, _) = ui.allocate_exact_size(size, egui::Sense::hover());
    let centre = rect.center();
    let project =
        |v: [f64; 3]| egui::vec2((v[0] - 0.6 * v[1]) as f32, (-v[2] + 0.35 * v[1]) as f32) * 75.;
    let painter = ui.painter_at(rect);
    for axis in [[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]] {
        painter.line_segment(
            [centre, centre + project(axis)],
            egui::Stroke::new(1., crate::theme::BORDER),
        );
    }
    for (axis, label, color) in [
        ([1., 0., 0.], "Right", crate::theme::ACCENT),
        ([0., 1., 0.], "Forward", crate::theme::SUCCESS),
        ([0., 0., 1.], "Up", crate::theme::TEXT),
    ] {
        let q = head.pose.normalized();
        let r = q
            .multiply(Quaternion([0., axis[0], axis[1], axis[2]]))
            .multiply(q.conjugate());
        let direction = project([r.0[1], r.0[2], r.0[3]]);
        let color = if head.status == TrackingStatus::Active {
            color
        } else {
            crate::theme::MUTED
        };
        painter.arrow(centre, direction, egui::Stroke::new(2., color));
        painter.text(
            centre + direction * 1.15,
            egui::Align2::CENTER_CENTER,
            label,
            egui::FontId::proportional(12.),
            color,
        );
    }
    ui.small("Installation mapping applied. No recentering, smoothing or prediction.");
    ui.small("The diagram uses physical head axes; OSC quaternion values use the documented YXZ profile.");
}

pub fn diagnostics(ui: &mut egui::Ui, sample: Option<&Sample>) {
    let Some(p) = sample else {
        ui.label("No motion data.");
        return;
    };
    ui.label(format!("Orientation source: {:?}", p.orientation_source));
    ui.label(format!(
        "Instance {} · session {} · reference {}",
        p.instance, p.session, p.reference
    ));
    if let Some(w) = p.gyro {
        ui.label(format!(
            "Head X/Y/Z gyro: {:.2}, {:.2}, {:.2} °/s",
            w[0].to_degrees(),
            w[1].to_degrees(),
            w[2].to_degrees()
        ));
    } else {
        ui.label("Same-frame angular velocity: unavailable");
    }
    if let Some([x, y, z]) = p.acceleration {
        ui.label(format!("Head X/Y/Z acceleration: {x:.3}, {y:.3}, {z:.3} g"));
    } else {
        ui.label("Same-frame acceleration: unavailable");
    }
    if let Some(t) = p.sample_time {
        ui.label(format!(
            "Sample time: {:?} · {} ms · epoch {}",
            t.kind, t.time_ms, t.clock_epoch
        ));
    } else {
        ui.label("Sample time: unavailable");
    }
    ui.small("Device time is not synchronized to host time. No timestamp is inferred from another frame.");
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn physical_rotation_is_not_the_wire_quaternion_and_pitch_is_not_clamped() {
        for angles in [
            [90., 0., 0.],
            [0., 89., 0.],
            [30., 20., 10.],
            [179., 0., 0.],
            [-179., 0., 0.],
        ] {
            let physical = Quaternion::from_euler(angles);
            let observed = physical.euler();
            for (a, b) in observed.into_iter().zip(angles) {
                assert!((a - b).abs() < 0.001);
            }
            let wire = pb::pose::from_euler(angles.map(f64::from)).unwrap();
            let converted = pb::pose::from_euler(observed.map(f64::from)).unwrap();
            assert!(pb::pose::angular_distance_deg(wire, converted).unwrap() < 0.001);
        }
        let q = Quaternion::from_euler([90., 0., 0.]);
        assert!(q.0[3].abs() > 0.7); // physical yaw rotates about Z, not wire Y
    }
    #[test]
    fn snapshot_never_references_or_smooths_and_duplicate_queries_expire() {
        let now = Instant::now();
        let p = Sample {
            instance: 1,
            session: 1,
            reference: 1,
            sequence: 1,
            physical: Quaternion::from_euler([37., 89., -20.]),
            angles: [37., 89., -20.],
            fresh: true,
            age: Duration::ZERO,
            queried_at: now,
            gyro: None,
            acceleration: None,
            sample_time: None,
            orientation_source: pb::OrientationSource::NativeQuaternion,
        };
        let active = TrackingSnapshot::from_sample(Some(&p), now);
        assert_eq!(active.pose, p.physical);
        assert_eq!(active.status, TrackingStatus::Active);
        let stale = TrackingSnapshot::from_sample(Some(&p), now + Duration::from_millis(500));
        assert_eq!(stale.pose, p.physical);
        assert_eq!(stale.status, TrackingStatus::Stale);
        assert!(stale.measurement.is_none());
    }
}
