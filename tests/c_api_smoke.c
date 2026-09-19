#include "posebridge.h"
#include <assert.h>
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#ifdef _WIN32
#include <windows.h>
static void wait_ms(unsigned ms) { Sleep(ms); }
#else
#include <time.h>
static void wait_ms(unsigned ms) { struct timespec ts = { ms / 1000, (long)(ms % 1000) * 1000000L }; nanosleep(&ts, NULL); }
#endif
int main(void) {
    assert(PB_ABI_VERSION == 300 && pb_abi_version() == PB_ABI_VERSION);
    assert(sizeof(PbPose) == 184 && sizeof(PbStatus) == 136);
    assert(offsetof(PbPose, instance_id) == 16 && offsetof(PbPose, quaternion_xyzw) == 80);
    assert(pb_context_create(NULL) == PB_INVALID_ARGUMENT);
    PbContext *ctx = NULL; assert(pb_context_create(&ctx) == PB_OK);
    PbPose p = {0}; p.struct_size = sizeof(p);
    assert(pb_latest_pose(ctx, &p) == PB_NO_DATA);
    p.struct_size = 4; assert(pb_latest_pose(ctx, &p) == PB_BUFFER_TOO_SMALL && p.struct_size == 4);
    p.struct_size = sizeof(p);
    uint32_t required = 0; char error[512];
    assert(pb_error_copy(ctx, NULL, 0, &required) == PB_BUFFER_TOO_SMALL);
    assert(pb_error_copy(ctx, error, sizeof(error), &required) == PB_OK && strstr(error, "struct_size"));
    char unchanged[2] = "x";
    assert(pb_snapshot_json(ctx, unchanged, 1, &required) == PB_BUFFER_TOO_SMALL && unchanged[0] == 'x');
    char *json = (char*)malloc(required); assert(json);
    assert(pb_snapshot_json(ctx, json, required, &required) == PB_OK && strstr(json, "\"schema\":3")); free(json);
    const char *config = "{\"source_id\":\"head\",\"source\":{\"kind\":\"simulate\",\"euler_deg\":[30,20,10],\"rate_hz\":1,\"sample_clock\":true}}";
    assert(pb_configure(ctx, config, (uint32_t)strlen(config)) == PB_OK);
    assert(pb_inspect_start(ctx) == PB_INVALID_ARGUMENT);
    assert(pb_start(ctx) == PB_OK);
    int rc = PB_NO_DATA;
    for (int i = 0; i < 100 && rc == PB_NO_DATA; ++i) {wait_ms(5);rc=pb_latest_pose(ctx,&p);}
    assert(rc == PB_OK && p.abi_version == 300 && p.fresh && p.sequence == 1);
    assert(p.sample_time_kind == 2 && p.sample_clock_epoch == 1 && p.sample_time_ms == p.received_ns / 1000000);
    assert(p.instance_id > 0 && p.reference_epoch > 0 && p.metadata_revision > 0);
    assert(fabsf(p.euler_yaw_pitch_roll_deg[0] - 30.0f) < 0.001f);
    uint64_t sequence = p.sequence, instance = p.instance_id;
    assert(pb_latest_pose(ctx, &p) == PB_OK && p.sequence == sequence);
    assert(pb_start(ctx) == PB_BUSY && pb_inspect_start(ctx) == PB_BUSY);
    const char *command = "{\"action\":\"rate\",\"hz\":100}";
    assert(pb_device_command(ctx, command, (uint32_t)strlen(command)) == PB_BUSY);
    char state[8192]; assert(pb_snapshot_json(ctx,state,sizeof(state),&required)==PB_OK);
    assert(strstr(state,"\"source_id\":\"head\"") && strstr(state,"\"sequence\":\"1\"") && strstr(state,"\"calibration_quality\":null"));
    PbStatus status = {0};status.struct_size=sizeof(status);assert(pb_status(ctx,&status)==PB_OK && status.state==3);
    wait_ms(550); assert(pb_latest_pose(ctx,&p)==PB_OK && p.fresh==0);
    assert(pb_stop(ctx)==PB_OK && pb_stop(ctx)==PB_OK);
    assert(pb_start(ctx)==PB_OK);
    for(int i=0;i<100;++i){wait_ms(5);if(pb_latest_pose(ctx,&p)==PB_OK && p.instance_id!=instance)break;}
    assert(p.instance_id!=instance && p.fresh);
    struct { PbPose value; uint64_t tail; } future = {0};future.value.struct_size=sizeof(future);future.tail=42;
    assert(pb_latest_pose(ctx,&future.value)==PB_OK && future.tail==42);
    assert(pb_context_destroy(ctx)==PB_OK && pb_context_destroy(NULL)==PB_OK);
    puts("PoseBridge unified C ABI: PASS");
    return 0;
}
