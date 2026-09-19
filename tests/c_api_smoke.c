#include "posebridge.h"
#include <assert.h>
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#ifdef _WIN32
#include <windows.h>
static void wait_ms(unsigned ms) { Sleep(ms); }
#else
#include <time.h>
static void wait_ms(unsigned ms) {
    struct timespec ts = { ms / 1000, (long)(ms % 1000) * 1000000L };
    nanosleep(&ts, NULL);
}
#endif

int main(void) {
    assert(pb_abi_version() == PB_ABI_VERSION);
    assert(sizeof(PbPose) == 136 && sizeof(PbPoseV2) == 160 && sizeof(PbStatus) == 96);
    assert(offsetof(PbPoseV2, pose) == 8 && offsetof(PbPoseV2, sample_time_ms) == 144);
    assert(offsetof(PbPose, struct_size) == 0);
    assert(offsetof(PbPose, session_id) == 8);
    assert(pb_context_create(NULL) == PB_INVALID_ARGUMENT);
    PbContext *ctx = NULL;
    assert(pb_context_create(&ctx) == PB_OK && ctx != NULL);
    PbPose p = {0}; p.struct_size = sizeof(p);
    assert(pb_latest_pose(ctx, &p) == PB_NO_DATA);
    PbPoseV2 v2 = {0}; v2.struct_size = sizeof(v2);
    assert(pb_latest_pose_v2(ctx, &v2) == PB_NO_DATA);
    assert(pb_latest_pose_v2(ctx, NULL) == PB_INVALID_ARGUMENT);
    v2.struct_size = 4;
    assert(pb_latest_pose_v2(ctx, &v2) == PB_BUFFER_TOO_SMALL && v2.struct_size == 4);
    v2.struct_size = sizeof(v2);
    PbPose undersized_pose = {0}; undersized_pose.struct_size = 4;
    assert(pb_latest_pose(ctx, &undersized_pose) == PB_BUFFER_TOO_SMALL);
    char error[512]; uint32_t required = 0;
    assert(pb_error_copy(ctx, NULL, 0, &required) == PB_BUFFER_TOO_SMALL);
    assert(required > 1);
    assert(pb_error_copy(ctx, error, sizeof(error), &required) == PB_OK);
    assert(strstr(error, "struct_size") != NULL);
    char devices[8] = "xxxxxxx";
    assert(pb_devices_json(ctx, devices, 1, &required) == PB_BUFFER_TOO_SMALL);
    assert(devices[0] == 'x' && required == 3);
    assert(pb_devices_json(ctx, devices, sizeof(devices), &required) == PB_OK);
    assert(strcmp(devices, "[]") == 0);
    const char *config = "{\"source\":{\"kind\":\"simulate\",\"pattern\":\"fixed\",\"euler_deg\":[30,20,10],\"rate_hz\":1}}";
    assert(pb_configure(ctx, config, (uint32_t)strlen(config)) == PB_OK);
    assert(pb_configure(ctx, "{broken}", 8) == PB_INVALID_ARGUMENT);
    assert(pb_start(ctx) == PB_OK);
    int rc = PB_NO_DATA;
    for (int i = 0; i < 100 && rc == PB_NO_DATA; ++i) {
        wait_ms(5); rc = pb_latest_pose(ctx, &p);
    }
    assert(rc == PB_OK && p.fresh == 1 && p.sequence == 1);
    assert(fabsf(p.euler_yaw_pitch_roll_deg[0] - 30.0f) < 0.0001f);
    assert(pb_latest_pose_v2(ctx, &v2) == PB_OK);
    assert(v2.sample_time_kind == 0 && v2.sample_time_ms == 0 && v2.sample_clock_epoch == 0);
    assert(v2.pose.sequence == p.sequence && v2.pose.received_ns == p.received_ns);
    uint64_t sequence = p.sequence, session = p.session_id;
    assert(pb_latest_pose(ctx, &p) == PB_OK && p.sequence == sequence);
    assert(pb_start(ctx) == PB_BUSY);
    PbStatus status = {0}; status.struct_size = sizeof(status);
    assert(pb_status(ctx, &status) == PB_OK && status.state == 3);
    assert(pb_stop(ctx) == PB_OK);
    assert(pb_stop(ctx) == PB_OK);
    assert(pb_latest_pose(ctx, &p) == PB_OK && p.fresh == 0);
    const char *timed = "{\"source\":{\"kind\":\"simulate\",\"rate_hz\":100,\"sample_clock\":true}}";
    assert(pb_configure(ctx, timed, (uint32_t)strlen(timed)) == PB_OK);
    assert(pb_start(ctx) == PB_OK);
    for (int i = 0; i < 100; ++i) {
        wait_ms(5);
        if (pb_latest_pose(ctx, &p) == PB_OK && p.session_id != session) break;
    }
    assert(p.session_id != session && p.fresh == 1);
    struct { PbPoseV2 value; uint64_t tail; } future = {0};
    future.value.struct_size = sizeof(future); future.tail = UINT64_C(0x123456789abcdef0);
    assert(pb_latest_pose_v2(ctx, &future.value) == PB_OK);
    assert(future.tail == UINT64_C(0x123456789abcdef0));
    assert(future.value.sample_time_kind == 2 && future.value.sample_clock_epoch == 1);
    assert(future.value.sample_time_ms == future.value.pose.received_ns / 1000000);
    assert(future.value.pose.struct_size == sizeof(PbPose));
    assert(pb_stop(ctx) == PB_OK);
    assert(pb_latest_pose_v2(ctx, &v2) == PB_OK && v2.pose.fresh == 0);
    assert(pb_context_destroy(ctx) == PB_OK);
    assert(pb_context_destroy(NULL) == PB_OK);
    puts("PoseBridge C ABI smoke: PASS");
    return 0;
}
