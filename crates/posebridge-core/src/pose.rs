//! The wire quaternion is the MacinRender GUI Y-X-Z profile, not a raw IMU quaternion.
use crate::{Error, Result};
use serde::{Deserialize, Serialize};

pub type Quat = [f64; 4];
type Matrix = [[f64; 3]; 3];

pub fn normalize(q: Quat) -> Result<Quat> {
    if q.iter().any(|v| !v.is_finite()) {
        return Err(Error::Invalid(
            "quaternion contains non-finite values".into(),
        ));
    }
    let n = q.iter().map(|v| v * v).sum::<f64>().sqrt();
    if !n.is_finite() || n < 1e-6 {
        return Err(Error::Invalid("quaternion norm is invalid".into()));
    }
    Ok(q.map(|v| v / n))
}

pub fn multiply(a: Quat, b: Quat) -> Quat {
    let [x, y, z, w] = a;
    let [xx, yy, zz, ww] = b;
    [
        w * xx + x * ww + y * zz - z * yy,
        w * yy - x * zz + y * ww + z * xx,
        w * zz + x * yy - y * xx + z * ww,
        w * ww - x * xx - y * yy - z * zz,
    ]
}

fn axis(index: usize, degrees: f64) -> Quat {
    let half = degrees.to_radians() / 2.0;
    let mut q = [0.0, 0.0, 0.0, half.cos()];
    q[index] = half.sin();
    q
}

pub fn from_euler([yaw, pitch, roll]: [f64; 3]) -> Result<Quat> {
    if [yaw, pitch, roll]
        .iter()
        .any(|v| !v.is_finite() || v.abs() > 1e6)
    {
        return Err(Error::Invalid(
            "Euler angles must be finite and within 1e6 degrees".into(),
        ));
    }
    normalize(multiply(
        multiply(axis(1, yaw), axis(0, pitch)),
        axis(2, roll),
    ))
}

pub fn to_euler(q: Quat) -> Result<[f64; 3]> {
    let [x, y, z, w] = normalize(q)?;
    Ok([
        (2.0 * (x * z + w * y))
            .atan2(1.0 - 2.0 * (x * x + y * y))
            .to_degrees(),
        (2.0 * (w * x - y * z)).clamp(-1.0, 1.0).asin().to_degrees(),
        (2.0 * (x * y + w * z))
            .atan2(1.0 - 2.0 * (x * x + z * z))
            .to_degrees(),
    ])
}

pub fn angular_distance_deg(a: Quat, b: Quat) -> Result<f64> {
    let a = normalize(a)?;
    let b = normalize(b)?;
    Ok(2.0
        * a.iter()
            .zip(b)
            .map(|(a, b)| a * b)
            .sum::<f64>()
            .abs()
            .clamp(0.0, 1.0)
            .acos()
            .to_degrees())
}

/// Signed sensor axes pointing towards head right/forward/up (1=X, 2=Y, 3=Z).
#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq, Eq)]
#[serde(deny_unknown_fields)]
pub struct Mounting {
    pub right: i8,
    pub forward: i8,
    pub up: i8,
}

impl Mounting {
    pub fn parse(text: &str) -> Result<Self> {
        let parts: Vec<_> = text.split(',').map(str::trim).collect();
        if parts.len() != 3 {
            return Err(Error::Invalid(
                "mount expects three signed axes, e.g. -y,+x,+z".into(),
            ));
        }
        let parse = |s: &str| -> Result<i8> {
            let lower = s.to_ascii_lowercase();
            let sign = if lower.starts_with('-') { -1 } else { 1 };
            let name = lower
                .strip_prefix('+')
                .or_else(|| lower.strip_prefix('-'))
                .unwrap_or(&lower);
            let axis = match name {
                "x" => 1,
                "y" => 2,
                "z" => 3,
                _ => return Err(Error::Invalid(format!("invalid axis {s}"))),
            };
            Ok(sign * axis)
        };
        let m = Self {
            right: parse(parts[0])?,
            forward: parse(parts[1])?,
            up: parse(parts[2])?,
        };
        m.validate()?;
        Ok(m)
    }

    pub fn validate(self) -> Result<()> {
        let values = [self.right, self.forward, self.up];
        let abs = values.map(i8::unsigned_abs);
        if abs.iter().any(|v| !(1..=3).contains(v))
            || abs[0] == abs[1]
            || abs[0] == abs[2]
            || abs[1] == abs[2]
        {
            return Err(Error::Invalid(
                "mount must use each sensor axis once".into(),
            ));
        }
        let m = self.matrix();
        let determinant = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
            - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
            + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
        if determinant < 0.5 {
            return Err(Error::Invalid("mount must be a right-handed basis".into()));
        }
        Ok(())
    }

    fn matrix(self) -> Matrix {
        let mut m = [[0.0; 3]; 3];
        for (row, axis) in [self.right, self.forward, self.up].into_iter().enumerate() {
            m[row][axis.unsigned_abs() as usize - 1] = if axis < 0 { -1.0 } else { 1.0 };
        }
        m
    }

    /// Re-express a vector in physical head X-right/Y-forward/Z-up axes.
    pub fn vector(self, sensor: [f64; 3]) -> Result<[f64; 3]> {
        self.validate()?;
        if sensor.iter().any(|v| !v.is_finite()) {
            return Err(Error::Invalid("non-finite motion vector".into()));
        }
        Ok([self.right, self.forward, self.up].map(|axis| {
            sensor[axis.unsigned_abs() as usize - 1] * if axis < 0 { -1.0 } else { 1.0 }
        }))
    }
    /// For a proper rotation M, conjugation M R M^T maps the quaternion vector
    /// by M while preserving its scalar. This returns a PHYSICAL quaternion.
    pub fn physical_from_sensor_quaternion(self, sensor: Quat) -> Result<Quat> {
        let [x, y, z, w] = normalize(sensor)?;
        let [x, y, z] = self.vector([x, y, z])?;
        Ok([x, y, z, w])
    }
    pub fn physical_from_sensor_euler(self, [x, y, z]: [f64; 3]) -> Result<Quat> {
        if [x, y, z].iter().any(|v| !v.is_finite() || v.abs() > 180.01) {
            return Err(Error::Protocol("invalid sensor Euler angles".into()));
        }
        self.physical_from_sensor_quaternion(multiply(multiply(axis(2, z), axis(1, y)), axis(0, x)))
    }

    /// WIT Euler XYZ is interpreted as Rz(Z) Ry(Y) Rx(X), then rotated as a full orientation.
    pub fn from_sensor_euler(self, [x, y, z]: [f64; 3]) -> Result<Quat> {
        if [x, y, z].iter().any(|v| !v.is_finite() || v.abs() > 180.01) {
            return Err(Error::Protocol("invalid sensor Euler angles".into()));
        }
        self.from_sensor_quaternion(multiply(multiply(axis(2, z), axis(1, y)), axis(0, x)))
    }

    pub fn from_sensor_quaternion(self, sensor_xyzw: Quat) -> Result<Quat> {
        self.validate()?;
        let [x, y, z, w] = normalize(sensor_xyzw)?;
        let r = [
            [
                1.0 - 2.0 * (y * y + z * z),
                2.0 * (x * y - w * z),
                2.0 * (x * z + w * y),
            ],
            [
                2.0 * (x * y + w * z),
                1.0 - 2.0 * (x * x + z * z),
                2.0 * (y * z - w * x),
            ],
            [
                2.0 * (x * z - w * y),
                2.0 * (y * z + w * x),
                1.0 - 2.0 * (x * x + y * y),
            ],
        ];
        let m = self.matrix();
        // Re-express both the sensor reference frame and sensor body frame in the mounting basis.
        let mut head = [[0.0; 3]; 3];
        for i in 0..3 {
            for j in 0..3 {
                for (a, row) in r.iter().enumerate() {
                    for (b, value) in row.iter().enumerate() {
                        head[i][j] += m[i][a] * value * m[j][b];
                    }
                }
            }
        }
        // Scene convention: Rz(yaw) Rx(pitch) Ry(roll). Convert to GUI YXZ only at the boundary.
        let pitch = head[2][1].clamp(-1.0, 1.0).asin();
        let (yaw, roll) = if pitch.cos().abs() > 1e-7 {
            (
                (-head[0][1]).atan2(head[1][1]),
                (-head[2][0]).atan2(head[2][2]),
            )
        } else {
            (head[1][0].atan2(head[0][0]), 0.0)
        };
        from_euler([yaw.to_degrees(), pitch.to_degrees(), roll.to_degrees()])
    }
}

/// Physical ZXY pose, distinct from the OSC/GUI YXZ representation.
pub fn physical_from_euler([yaw, pitch, roll]: [f64; 3]) -> Result<Quat> {
    if [yaw, pitch, roll].iter().any(|v| !v.is_finite()) {
        return Err(Error::Invalid("non-finite Euler angles".into()));
    }
    normalize(multiply(
        multiply(axis(2, yaw), axis(0, pitch)),
        axis(1, roll),
    ))
}
