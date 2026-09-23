//! Experimental C ABI for PoseBridge.
//! All non-null caller pointers must be live, aligned, and valid for their supplied lengths.
//! Lifecycle/configuration calls are externally serialized; destroy must not race any access.

use posebridge_core::{
    Config, Controller, DeviceCommand, Error, MagneticCursor, PoseSnapshot, TransportKind,
};
use std::ffi::c_char;
use std::panic::{catch_unwind, AssertUnwindSafe};
use std::ptr;
use std::sync::{Mutex, MutexGuard};

pub const PB_ABI_VERSION: u32 = 400;
pub const PB_OK: i32 = 0;
pub const PB_INVALID_ARGUMENT: i32 = 1;
pub const PB_NO_DATA: i32 = 2;
pub const PB_BUSY: i32 = 3;
pub const PB_UNAVAILABLE: i32 = 4;
pub const PB_PERMISSION: i32 = 5;
pub const PB_IO_ERROR: i32 = 6;
pub const PB_PROTOCOL_ERROR: i32 = 7;
pub const PB_TIMEOUT: i32 = 8;
pub const PB_BUFFER_TOO_SMALL: i32 = 9;
pub const PB_INTERNAL_ERROR: i32 = 10;
pub const PB_CANCELLED: i32 = 11;
pub const PB_TRANSPORT_BLE: u32 = 0;
pub const PB_TRANSPORT_USB: u32 = 1;

/// Opaque, context-owned runtime and device session. Never allocate this in the caller.
pub struct PbContext {
    controller: Mutex<Controller>,
    error: Mutex<String>,
}

/// Current ABI only. Initialize struct_size; check pb_abi_version before use.
/// Sample time has its own clock. Host received_ns is relative to the source session.
#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct PbPose {
    pub struct_size: u32,
    pub abi_version: u32,
    pub fresh: u32,
    pub sample_time_kind: u32,
    pub instance_id: u64,
    pub reference_epoch: u64,
    pub metadata_revision: u64,
    pub sample_time_ms: u64,
    pub sample_clock_epoch: u64,
    pub session_id: u64,
    pub sequence: u64,
    pub received_ns: u64,
    /// Host monotonic nanoseconds from reception to this query, including while stopped.
    /// Excludes device/link delay. fresh also requires an active acquisition state.
    pub age_ns: u64,
    pub quaternion_xyzw: [f32; 4],
    pub euler_yaw_pitch_roll_deg: [f32; 3],
    /// bit 0: complete motion group; bit 1: quaternion; bits 2/3/4: Euler/accel/gyro present.
    pub raw_flags: u32,
    pub raw_euler_xyz_deg: [f32; 3],
    pub acceleration_g: [f32; 3],
    pub angular_velocity_dps: [f32; 3],
    pub raw_quaternion_wxyz: [f32; 4],
    pub motion_received_ns: u64,
    pub quaternion_received_ns: u64,
}

/// Initialize struct_size to sizeof(PbStatus). State values are documented in docs/c-api.md.
#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct PbStatus {
    pub struct_size: u32,
    pub state: u32,
    pub session_id: u64,
    pub pose_count: u64,
    pub session_samples: u64,
    pub coalesced_samples: u64,
    pub send_errors: u64,
    pub telemetry_sent: u64,
    pub delivery_reads: u64,
    pub bytes_received: u64,
    pub frames_received: u64,
    pub discarded_bytes: u64,
    pub invalid_poses: u64,
    pub osc_sent: u64,
    pub reconnect_count: u64,
    pub actual_rate_hz: f64,
    pub interval_min_ms: f64,
    pub interval_max_ms: f64,
}

struct Failure(i32, String);
impl From<Error> for Failure {
    fn from(e: Error) -> Self {
        let code = match e {
            Error::Invalid(_) => PB_INVALID_ARGUMENT,
            Error::Busy => PB_BUSY,
            Error::Unavailable(_) => PB_UNAVAILABLE,
            Error::Permission(_) => PB_PERMISSION,
            Error::Io(_) => PB_IO_ERROR,
            Error::Protocol(_) => PB_PROTOCOL_ERROR,
            Error::Timeout(_) => PB_TIMEOUT,
            Error::Cancelled => PB_CANCELLED,
            Error::Internal(_) => PB_INTERNAL_ERROR,
        };
        Self(code, e.to_string())
    }
}
type FfiResult<T> = std::result::Result<T, Failure>;

fn lock<T>(value: &Mutex<T>) -> MutexGuard<'_, T> {
    value.lock().unwrap_or_else(|p| p.into_inner())
}

fn boundary(context: *const PbContext, operation: impl FnOnce() -> FfiResult<()>) -> i32 {
    let result = catch_unwind(AssertUnwindSafe(operation)).unwrap_or_else(|_| {
        Err(Failure(
            PB_INTERNAL_ERROR,
            "panic caught at C boundary".into(),
        ))
    });
    match result {
        Ok(()) => PB_OK,
        Err(Failure(code, message)) => {
            if !context.is_null() {
                // SAFETY: exported function contract requires a valid context; destroy passes NULL.
                unsafe {
                    *lock(&(*context).error) = message;
                }
            }
            code
        }
    }
}

unsafe fn context<'a>(value: *const PbContext) -> FfiResult<&'a PbContext> {
    value
        .as_ref()
        .ok_or_else(|| Failure(PB_INVALID_ARGUMENT, "context is NULL".into()))
}

unsafe fn json_input<'a>(data: *const c_char, len: u32) -> FfiResult<&'a str> {
    if data.is_null() || len == 0 || len > 65536 {
        return Err(Failure(
            PB_INVALID_ARGUMENT,
            "JSON pointer/length invalid (maximum 65536 bytes)".into(),
        ));
    }
    std::str::from_utf8(std::slice::from_raw_parts(data.cast::<u8>(), len as usize))
        .map_err(|e| Failure(PB_INVALID_ARGUMENT, e.to_string()))
}

unsafe fn copy_text(
    value: &str,
    buffer: *mut c_char,
    capacity: u32,
    required: *mut u32,
) -> FfiResult<()> {
    if required.is_null() {
        return Err(Failure(
            PB_INVALID_ARGUMENT,
            "required pointer is NULL".into(),
        ));
    }
    let needed = u32::try_from(value.len() + 1)
        .map_err(|_| Failure(PB_INTERNAL_ERROR, "output exceeds ABI buffer limit".into()))?;
    *required = needed;
    if capacity < needed {
        return Err(Failure(
            PB_BUFFER_TOO_SMALL,
            "buffer too small; required includes NUL".into(),
        ));
    }
    if buffer.is_null() {
        return Err(Failure(PB_INVALID_ARGUMENT, "buffer is NULL".into()));
    }
    ptr::copy_nonoverlapping(value.as_ptr(), buffer.cast::<u8>(), value.len());
    *buffer.add(value.len()) = 0;
    Ok(())
}

#[no_mangle]
pub extern "C" fn pb_abi_version() -> u32 {
    PB_ABI_VERSION
}

/// Create a context. On failure, *out is NULL.
/// # Safety
/// out points to writable pointer storage.
#[no_mangle]
pub unsafe extern "C" fn pb_context_create(out: *mut *mut PbContext) -> i32 {
    boundary(ptr::null(), || {
        if out.is_null() {
            return Err(Failure(PB_INVALID_ARGUMENT, "out is NULL".into()));
        }
        *out = ptr::null_mut();
        let value = PbContext {
            controller: Mutex::new(Controller::new()?),
            error: Mutex::new(String::new()),
        };
        *out = Box::into_raw(Box::new(value));
        Ok(())
    })
}

/// Stop and consume the context even if stop reports an error. NULL is allowed.
/// # Safety
/// The context is live, exclusively owned by this call, and is destroyed exactly once.
#[no_mangle]
pub unsafe extern "C" fn pb_context_destroy(value: *mut PbContext) -> i32 {
    boundary(ptr::null(), || {
        if value.is_null() {
            return Ok(());
        }
        let owner = Box::from_raw(value);
        let result = lock(&owner.controller).stop();
        drop(owner);
        result?;
        Ok(())
    })
}

/// Copy and validate a Config JSON document. Only allowed while stopped.
/// # Safety
/// context is live and data references len readable UTF-8 bytes; no overlapping lifecycle calls.
#[no_mangle]
pub unsafe extern "C" fn pb_configure(value: *mut PbContext, data: *const c_char, len: u32) -> i32 {
    boundary(value, || {
        let ctx = context(value)?;
        let config: Config = serde_json::from_str(json_input(data, len)?)
            .map_err(|e| Failure(PB_INVALID_ARGUMENT, e.to_string()))?;
        lock(&ctx.controller).set_config(config)?;
        Ok(())
    })
}

/// Start asynchronous acquisition; PB_OK means accepted, not connected.
/// # Safety
/// value is live and lifecycle/configuration calls are serialized.
#[no_mangle]
pub unsafe extern "C" fn pb_start(value: *mut PbContext) -> i32 {
    boundary(value, || {
        lock(&context(value)?.controller).start()?;
        Ok(())
    })
}

/// Stop scanning, acquisition, configuration or reconnect; may wait for resource cleanup.
/// # Safety
/// value is live and lifecycle/configuration calls are serialized.
#[no_mangle]
pub unsafe extern "C" fn pb_stop(value: *mut PbContext) -> i32 {
    boundary(value, || {
        lock(&context(value)?.controller).stop()?;
        Ok(())
    })
}

/// Start device enumeration. Read completion and failure through pb_status/pb_snapshot_json.
/// # Safety
/// value is live and lifecycle/configuration calls are serialized.
#[no_mangle]
pub unsafe extern "C" fn pb_scan_start(value: *mut PbContext, transport: u32, seconds: u32) -> i32 {
    boundary(value, || {
        let kind = match transport {
            PB_TRANSPORT_BLE => TransportKind::Ble,
            PB_TRANSPORT_USB => TransportKind::Usb,
            _ => return Err(Failure(PB_INVALID_ARGUMENT, "unknown transport".into())),
        };
        lock(&context(value)?.controller).scan_start(kind, seconds)?;
        Ok(())
    })
}

/// Start a read-only hardware inspection while idle. Poll pb_snapshot_json for completion.
/// # Safety
/// value is live and lifecycle/configuration calls are serialized.
#[no_mangle]
pub unsafe extern "C" fn pb_inspect_start(value: *mut PbContext) -> i32 {
    boundary(value, || {
        lock(&context(value)?.controller).inspect_start()?;
        Ok(())
    })
}

/// Open an exclusive read-only magnetic session (PoseBridge >= 0.6).
/// No mounting is required. Use pb_device_command for mag_start/mag_stop/save,
/// and pb_stop to close. Normal acquisition must already be stopped.
/// # Safety
/// value is live and lifecycle/control calls are serialized.
#[no_mangle]
pub unsafe extern "C" fn pb_magnetic_start(value: *mut PbContext) -> i32 {
    boundary(value, || {
        lock(&context(value)?.controller).magnetic_start()?;
        Ok(())
    })
}

/// Query a non-consuming magnetic batch, status, statistics and operation result
/// as JSON schema 1. cursor=NULL with cursor_len=0 (or JSON null) reads retained
/// history; otherwise pass the returned cursor object. 64-bit values are strings.
/// Like pb_snapshot_json, size queries return PB_BUFFER_TOO_SMALL; retrying never
/// consumes records. Histories remain queryable after stop, with fresh=false.
/// # Safety
/// value is live; cursor points to cursor_len bytes (NULL only when length=0);
/// required is writable; buffer is writable for capacity bytes (NULL iff capacity=0).
#[no_mangle]
pub unsafe extern "C" fn pb_magnetic_since_json(
    value: *const PbContext,
    cursor: *const c_char,
    cursor_len: u32,
    buffer: *mut c_char,
    capacity: u32,
    required: *mut u32,
) -> i32 {
    boundary(value, || {
        let cursor: Option<MagneticCursor> = if cursor.is_null() && cursor_len == 0 {
            None
        } else {
            serde_json::from_str(json_input(cursor, cursor_len)?)
                .map_err(|e| Failure(PB_INVALID_ARGUMENT, e.to_string()))?
        };
        let batch = lock(&context(value)?.controller).magnetic_since(cursor);
        let text =
            serde_json::to_string(&batch).map_err(|e| Failure(PB_INTERNAL_ERROR, e.to_string()))?;
        copy_text(&text, buffer, capacity, required)
    })
}

/// Issue an explicit device command JSON (e.g. {"action":"rate","hz":100}).
/// # Safety
/// value is live, data references len readable bytes, and control calls are serialized.
#[no_mangle]
pub unsafe extern "C" fn pb_device_command(
    value: *mut PbContext,
    data: *const c_char,
    len: u32,
) -> i32 {
    boundary(value, || {
        let command: DeviceCommand = serde_json::from_str(json_input(data, len)?)
            .map_err(|e| Failure(PB_INVALID_ARGUMENT, e.to_string()))?;
        lock(&context(value)?.controller).configure_device(command)?;
        Ok(())
    })
}

fn pose_to_c(p: &PoseSnapshot) -> PbPose {
    let raw_flags = u32::from(
        p.raw.euler_xyz_deg.is_some()
            && p.raw.acceleration_g.is_some()
            && p.raw.angular_velocity_dps.is_some(),
    ) | (u32::from(p.raw.quaternion_wxyz.is_some()) << 1)
        | (u32::from(p.raw.euler_xyz_deg.is_some()) << 2)
        | (u32::from(p.raw.acceleration_g.is_some()) << 3)
        | (u32::from(p.raw.angular_velocity_dps.is_some()) << 4);
    PbPose {
        struct_size: std::mem::size_of::<PbPose>() as u32,
        abi_version: PB_ABI_VERSION,
        instance_id: p.instance_id,
        reference_epoch: p.reference_epoch,
        metadata_revision: p.metadata_revision,
        sample_time_kind: p.sample_time.map_or(0, |t| t.kind as u32),
        sample_time_ms: p.sample_time.map_or(0, |t| t.time_ms),
        sample_clock_epoch: p.sample_time.map_or(0, |t| t.clock_epoch),
        fresh: u32::from(p.fresh),
        session_id: p.session_id,
        sequence: p.sequence,
        received_ns: p.received_ns,
        age_ns: p.age_ns,
        quaternion_xyzw: p.quaternion_xyzw.map(|v| v as f32),
        euler_yaw_pitch_roll_deg: p.euler_deg.map(|v| v as f32),
        raw_flags,
        raw_euler_xyz_deg: p.raw.euler_xyz_deg.unwrap_or_default().map(|v| v as f32),
        acceleration_g: p.raw.acceleration_g.unwrap_or_default().map(|v| v as f32),
        angular_velocity_dps: p
            .raw
            .angular_velocity_dps
            .unwrap_or_default()
            .map(|v| v as f32),
        raw_quaternion_wxyz: p.raw.quaternion_wxyz.unwrap_or_default().map(|v| v as f32),
        motion_received_ns: p.raw.motion_received_ns.unwrap_or_default(),
        quaternion_received_ns: p.raw.quaternion_received_ns.unwrap_or_default(),
    }
}

/// Copy the latest snapshot without waiting for new data. PB_NO_DATA before first pose.
/// # Safety
/// value is live and out is aligned writable PbPose storage with struct_size initialized.
#[no_mangle]
pub unsafe extern "C" fn pb_latest_pose(value: *const PbContext, out: *mut PbPose) -> i32 {
    boundary(value, || {
        let ctx = context(value)?;
        if out.is_null() {
            return Err(Failure(PB_INVALID_ARGUMENT, "out is NULL".into()));
        }
        if (*out).struct_size < std::mem::size_of::<PbPose>() as u32 {
            return Err(Failure(
                PB_BUFFER_TOO_SMALL,
                "PbPose struct_size too small".into(),
            ));
        }
        let p = lock(&ctx.controller)
            .latest_pose()
            .ok_or_else(|| Failure(PB_NO_DATA, "no pose received yet".into()))?;
        *out = pose_to_c(&p);
        Ok(())
    })
}

/// Copy current state and counters. State: 0 idle, 1 scanning, 2 connecting, 3 active, 4 stale,
/// 5 reconnecting, 6 stopped, 7 failed, 8 configuring, 9 complete, 10 inspecting.
/// # Safety
/// value is live and out is aligned writable PbStatus storage with struct_size initialized.
#[no_mangle]
pub unsafe extern "C" fn pb_status(value: *const PbContext, out: *mut PbStatus) -> i32 {
    boundary(value, || {
        let ctx = context(value)?;
        if out.is_null() {
            return Err(Failure(PB_INVALID_ARGUMENT, "out is NULL".into()));
        }
        if (*out).struct_size < std::mem::size_of::<PbStatus>() as u32 {
            return Err(Failure(
                PB_BUFFER_TOO_SMALL,
                "PbStatus struct_size too small".into(),
            ));
        }
        let s = lock(&ctx.controller).status();
        *out = PbStatus {
            struct_size: std::mem::size_of::<PbStatus>() as u32,
            state: s.state as u32,
            session_id: s.session_id,
            pose_count: s.pose_count,
            session_samples: s.session_samples,
            coalesced_samples: s.coalesced_samples,
            send_errors: s.send_errors,
            telemetry_sent: s.telemetry_sent,
            delivery_reads: s.delivery.reads,
            bytes_received: s.bytes_received,
            frames_received: s.frames_received,
            discarded_bytes: s.discarded_bytes,
            invalid_poses: s.invalid_poses,
            osc_sent: s.osc_sent,
            reconnect_count: s.reconnect_count,
            actual_rate_hz: s.actual_rate_hz,
            interval_min_ms: s.interval_min_ms,
            interval_max_ms: s.interval_max_ms,
        };
        Ok(())
    })
}

/// Copy the discovered device list as UTF-8 JSON. required includes the NUL terminator.
/// # Safety
/// value is live; required is writable; buffer is writable for capacity bytes (NULL iff capacity=0).
#[no_mangle]
pub unsafe extern "C" fn pb_devices_json(
    value: *const PbContext,
    buffer: *mut c_char,
    capacity: u32,
    required: *mut u32,
) -> i32 {
    boundary(value, || {
        let devices = lock(&context(value)?.controller).devices();
        let text = serde_json::to_string(&devices)
            .map_err(|e| Failure(PB_INTERNAL_ERROR, e.to_string()))?;
        copy_text(&text, buffer, capacity, required)
    })
}

/// Atomically copy source information, observation, pose, counters and operation result as UTF-8 JSON.
/// # Safety
/// value is live; required is writable; buffer is writable for capacity bytes (NULL iff capacity=0).
#[no_mangle]
pub unsafe extern "C" fn pb_snapshot_json(
    value: *const PbContext,
    buffer: *mut c_char,
    capacity: u32,
    required: *mut u32,
) -> i32 {
    boundary(value, || {
        let snapshot = lock(&context(value)?.controller).snapshot();
        let text = posebridge_core::snapshot_json(&snapshot)?;
        copy_text(&text, buffer, capacity, required)
    })
}

/// Copy the last synchronous API error. Querying does not overwrite it, including size queries.
/// # Safety
/// value is live; required is writable; buffer is writable for capacity bytes (NULL iff capacity=0).
#[no_mangle]
pub unsafe extern "C" fn pb_error_copy(
    value: *const PbContext,
    buffer: *mut c_char,
    capacity: u32,
    required: *mut u32,
) -> i32 {
    boundary(ptr::null(), || {
        let text = lock(&context(value)?.error).clone();
        copy_text(&text, buffer, capacity, required)
    })
}
