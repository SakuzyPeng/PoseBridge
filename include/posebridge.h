#ifndef POSEBRIDGE_H
#define POSEBRIDGE_H

#include <stdarg.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>

#define PB_ABI_VERSION 200

#define PB_OK 0

#define PB_INVALID_ARGUMENT 1

#define PB_NO_DATA 2

#define PB_BUSY 3

#define PB_UNAVAILABLE 4

#define PB_PERMISSION 5

#define PB_IO_ERROR 6

#define PB_PROTOCOL_ERROR 7

#define PB_TIMEOUT 8

#define PB_BUFFER_TOO_SMALL 9

#define PB_INTERNAL_ERROR 10

#define PB_CANCELLED 11

#define PB_TRANSPORT_BLE 0

#define PB_TRANSPORT_USB 1

/**
 * Opaque, context-owned runtime and device session. Never allocate this in the caller.
 */
typedef struct PbContext PbContext;

/**
 * Initialize struct_size to sizeof(PbPose). All timestamps are session-relative host times.
 */
typedef struct PbPose {
  uint32_t struct_size;
  uint32_t fresh;
  uint64_t session_id;
  uint64_t sequence;
  uint64_t received_ns;
  float quaternion_xyzw[4];
  float euler_yaw_pitch_roll_deg[3];
  /**
   * bit 0: complete motion group; bit 1: quaternion; bits 2/3/4: Euler/accel/gyro present.
   */
  uint32_t raw_flags;
  float raw_euler_xyz_deg[3];
  float acceleration_g[3];
  float angular_velocity_dps[3];
  float raw_quaternion_wxyz[4];
  uint64_t motion_received_ns;
  uint64_t quaternion_received_ns;
} PbPose;

/**
 * Experimental additive snapshot. Initialize only the outer struct_size.
 * pose.received_ns is host monotonic time; sample_time_ms uses its own clock.
 * kind 0: absent (time/epoch=0); 1: device calendar since 2000-01-01, NOT UTC;
 * 2: synthetic elapsed time. Present clocks have nonzero epoch; compare within
 * the same pose.session_id, kind and epoch only. Never subtract different clocks.
 */
typedef struct PbPoseV2 {
  uint32_t struct_size;
  uint32_t sample_time_kind;
  struct PbPose pose;
  uint64_t sample_time_ms;
  uint64_t sample_clock_epoch;
} PbPoseV2;

/**
 * Initialize struct_size to sizeof(PbStatus). State values are documented in docs/c-api.md.
 */
typedef struct PbStatus {
  uint32_t struct_size;
  uint32_t state;
  uint64_t session_id;
  uint64_t pose_count;
  uint64_t bytes_received;
  uint64_t frames_received;
  uint64_t discarded_bytes;
  uint64_t invalid_poses;
  uint64_t osc_sent;
  uint64_t reconnect_count;
  double actual_rate_hz;
  double interval_min_ms;
  double interval_max_ms;
} PbStatus;

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

uint32_t pb_abi_version(void);

/**
 * Create a context. On failure, *out is NULL.
 * # Safety
 * out points to writable pointer storage.
 */
int32_t pb_context_create(struct PbContext **out);

/**
 * Stop and consume the context even if stop reports an error. NULL is allowed.
 * # Safety
 * The context is live, exclusively owned by this call, and is destroyed exactly once.
 */
int32_t pb_context_destroy(struct PbContext *value);

/**
 * Copy and validate a Config JSON document. Only allowed while stopped.
 * # Safety
 * context is live and data references len readable UTF-8 bytes; no overlapping lifecycle calls.
 */
int32_t pb_configure(struct PbContext *value, const char *data, uint32_t len);

/**
 * Start asynchronous acquisition; PB_OK means accepted, not connected.
 * # Safety
 * value is live and lifecycle/configuration calls are serialized.
 */
int32_t pb_start(struct PbContext *value);

/**
 * Stop scanning, acquisition, configuration or reconnect; may wait for resource cleanup.
 * # Safety
 * value is live and lifecycle/configuration calls are serialized.
 */
int32_t pb_stop(struct PbContext *value);

/**
 * Start device enumeration. Read completion and failure through pb_status/pb_status_json.
 * # Safety
 * value is live and lifecycle/configuration calls are serialized.
 */
int32_t pb_scan_start(struct PbContext *value, uint32_t transport, uint32_t seconds);

/**
 * Issue an explicit device command JSON (e.g. {"action":"rate","hz":100}).
 * # Safety
 * value is live, data references len readable bytes, and control calls are serialized.
 */
int32_t pb_device_command(struct PbContext *value, const char *data, uint32_t len);

/**
 * Copy pose and sample time atomically from one acquisition snapshot.
 * PB_NO_DATA before the first valid pose; undersized outputs are unchanged.
 * # Safety
 * value is live; out is aligned writable PbPoseV2 storage with struct_size initialized.
 */
int32_t pb_latest_pose_v2(const struct PbContext *value, struct PbPoseV2 *out);

/**
 * Copy the latest snapshot without waiting for new data. PB_NO_DATA before first pose.
 * # Safety
 * value is live and out is aligned writable PbPose storage with struct_size initialized.
 */
int32_t pb_latest_pose(const struct PbContext *value, struct PbPose *out);

/**
 * Copy current state and counters. State: 0 idle, 1 scanning, 2 connecting, 3 active, 4 stale,
 * 5 reconnecting, 6 stopped, 7 failed, 8 configuring, 9 complete.
 * # Safety
 * value is live and out is aligned writable PbStatus storage with struct_size initialized.
 */
int32_t pb_status(const struct PbContext *value, struct PbStatus *out);

/**
 * Copy the discovered device list as UTF-8 JSON. required includes the NUL terminator.
 * # Safety
 * value is live; required is writable; buffer is writable for capacity bytes (NULL iff capacity=0).
 */
int32_t pb_devices_json(const struct PbContext *value,
                        char *buffer,
                        uint32_t capacity,
                        uint32_t *required);

/**
 * Copy state, asynchronous error and configuration report as UTF-8 JSON.
 * # Safety
 * value is live; required is writable; buffer is writable for capacity bytes (NULL iff capacity=0).
 */
int32_t pb_status_json(const struct PbContext *value,
                       char *buffer,
                       uint32_t capacity,
                       uint32_t *required);

/**
 * Copy the last synchronous API error. Querying does not overwrite it, including size queries.
 * # Safety
 * value is live; required is writable; buffer is writable for capacity bytes (NULL iff capacity=0).
 */
int32_t pb_error_copy(const struct PbContext *value,
                      char *buffer,
                      uint32_t capacity,
                      uint32_t *required);

#ifdef __cplusplus
}  // extern "C"
#endif  // __cplusplus

#endif  /* POSEBRIDGE_H */
