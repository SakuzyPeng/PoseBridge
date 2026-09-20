/* Standalone C11 control-thread consumer. Never call this loop from an audio callback.
 * Default: simulator only, no OSC or device operations. See docs/consumer.md.
 */
#ifndef _WIN32
#define _POSIX_C_SOURCE 200809L
#else
#define _CRT_SECURE_NO_WARNINGS
#endif
#include "pose_consumer_policy.h"
#include <errno.h>
#include <inttypes.h>
#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#ifdef _WIN32
#include <windows.h>
#else
#include <time.h>
#endif

static volatile sig_atomic_t interrupted = 0;
static void on_signal(int number) { (void)number; interrupted = 1; }

static void wait_ms(unsigned ms) {
#ifdef _WIN32
    Sleep(ms);
#else
    struct timespec delay = {ms / 1000U, (long)(ms % 1000U) * 1000000L};
    /* An interrupted sleep should return to the cancellation check promptly. */
    nanosleep(&delay, NULL);
#endif
}

static int monotonic_seconds(double *value) {
#ifdef _WIN32
    LARGE_INTEGER counter, frequency;
    if (!QueryPerformanceCounter(&counter) || !QueryPerformanceFrequency(&frequency)) return 0;
    *value = (double)counter.QuadPart / (double)frequency.QuadPart;
#else
    struct timespec now;
    if (clock_gettime(CLOCK_MONOTONIC, &now) != 0) return 0;
    *value = (double)now.tv_sec + (double)now.tv_nsec / 1e9;
#endif
    return 1;
}

static void on_reference_change(const PbPose *pose) {
    /* The host decides whether to preserve or reacquire its listening-forward reference. */
    printf("REFERENCE instance=%" PRIu64 " session=%" PRIu64 " epoch=%" PRIu64 "\n",
           pose->instance_id, pose->session_id, pose->reference_epoch);
}

static void on_pose(const PbPose *pose) {
    /* Send this target to the host's recenter/smoothing and presentation pipeline.
     * Use a host-owned real-time-safe handoff if an audio thread consumes it.
     * Query age applies at the getter; allow for any subsequent host queueing too. */
    (void)pose;
}

static int print_snapshot(PbContext *ctx, FILE *output) {
    char *buffer = NULL;
    uint32_t capacity = 0, required = 0;
    for (unsigned attempt = 0; attempt < 5; ++attempt) {
        const int rc = pb_snapshot_json(ctx, buffer, capacity, &required);
        if (rc == PB_OK) {
            const int ok = fprintf(output, "%s\n", buffer) >= 0;
            free(buffer);
            return ok;
        }
        if (rc != PB_BUFFER_TOO_SMALL || required <= capacity) break;
        char *grown = (char *)realloc(buffer, required);
        if (!grown) break;
        buffer = grown;
        capacity = required;
    }
    free(buffer);
    fputs("Could not copy diagnostic snapshot\n", stderr);
    return 0;
}

static void report_error(PbContext *ctx, const char *operation, int rc) {
    char message[1024] = {0};
    uint32_t required = 0;
    if (ctx && pb_error_copy(ctx, message, sizeof(message), &required) == PB_OK) {
        fprintf(stderr, "%s failed (%d): %s\n", operation, rc, message);
    } else {
        fprintf(stderr, "%s failed (%d)\n", operation, rc);
    }
}

static char *read_config(const char *path, uint32_t *length) {
    FILE *file = fopen(path, "rb");
    if (!file) { perror(path); return NULL; }
    char *data = (char *)malloc(65537);
    if (!data) { fclose(file); return NULL; }
    const size_t count = fread(data, 1, 65537, file);
    const int failed = ferror(file);
    fclose(file);
    if (failed || count == 0 || count > 65536) {
        fputs("Config must contain 1..65536 bytes of UTF-8 JSON\n", stderr);
        free(data);
        return NULL;
    }
    *length = (uint32_t)count;
    return data; /* pb_configure uses the explicit length; no NUL is required. */
}

static void usage(FILE *output) {
    fputs("pose_consumer [--config FILE] [--duration SECONDS] [--max-age-ms 1..500]\n"
          "Defaults: 100 Hz simulator, no OSC, 3 seconds, 100 ms age limit.\n", output);
}

int main(int argc, char **argv) {
    const char *config_path = NULL;
    double duration = 3.0;
    uint64_t max_age_ns = UINT64_C(100000000);
    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "--help") == 0) { usage(stdout); return 0; }
        if (i + 1 == argc) { usage(stderr); return 1; }
        const char *option = argv[i], *value = argv[++i];
        char *end = NULL;
        errno = 0;
        if (strcmp(option, "--config") == 0) {
            config_path = value;
        } else if (strcmp(option, "--duration") == 0) {
            duration = strtod(value, &end);
            if (errno || end == value || *end || !isfinite(duration) || duration <= 0) {
                usage(stderr); return 1;
            }
        } else if (strcmp(option, "--max-age-ms") == 0) {
            const unsigned long ms = strtoul(value, &end, 10);
            if (errno || end == value || *end || ms == 0 || ms > 500) { usage(stderr); return 1; }
            max_age_ns = (uint64_t)ms * UINT64_C(1000000);
        } else {
            usage(stderr); return 1;
        }
    }
    if (pb_abi_version() != PB_ABI_VERSION) {
        fputs("PoseBridge ABI mismatch: use the matching header and library\n", stderr);
        return 1;
    }
    const char *config = "{\"source_id\":\"c-consumer\",\"source\":{\"kind\":\"simulate\","
        "\"pattern\":\"combined\",\"rate_hz\":100,\"sample_clock\":true},\"osc\":null}";
    uint32_t config_length = (uint32_t)strlen(config);
    char *owned_config = NULL;
    if (config_path) {
        owned_config = read_config(config_path, &config_length);
        if (!owned_config) return 1;
        config = owned_config;
    }
    if (signal(SIGINT, on_signal) == SIG_ERR || signal(SIGTERM, on_signal) == SIG_ERR) {
        free(owned_config);
        return 1;
    }
    PbContext *ctx = NULL;
    ConsumerState consumer = {0};
    uint64_t updates = 0, freezes = 0, references = 0;
    int exit_code = 1;
    int rc = pb_context_create(&ctx);
    double start = 0;
    if (rc != PB_OK) { report_error(ctx, "create", rc); goto cleanup; }
    rc = pb_configure(ctx, config, config_length);
    if (rc != PB_OK) { report_error(ctx, "configure", rc); goto cleanup; }
    rc = pb_start(ctx);
    if (rc != PB_OK) { report_error(ctx, "start", rc); goto cleanup; }
    if (!monotonic_seconds(&start)) { fputs("Monotonic clock unavailable\n", stderr); goto cleanup; }
    puts("READY"); fflush(stdout);
    while (!interrupted) {
        double now = 0;
        if (!monotonic_seconds(&now)) { fputs("Monotonic clock unavailable\n", stderr); goto cleanup; }
        if (now - start >= duration) { exit_code = updates ? 0 : 2; goto cleanup; }
        PbPose pose = {0}; pose.struct_size = sizeof(pose);
        rc = pb_latest_pose(ctx, &pose);
        if (rc != PB_OK && rc != PB_NO_DATA) { report_error(ctx, "latest_pose", rc); goto cleanup; }
        const unsigned events = consume_pose(&consumer, rc == PB_OK ? &pose : NULL, max_age_ns);
        if (events & CONSUMER_REFERENCE) { ++references; on_reference_change(&pose); }
        if (events & CONSUMER_RESUMED) puts("ACTIVE");
        if (events & CONSUMER_POSE) { ++updates; on_pose(&pose); }
        if (events & CONSUMER_FROZEN) { ++freezes; puts("FROZEN: keep the last presented orientation"); }
        PbStatus status = {0}; status.struct_size = sizeof(status);
        rc = pb_status(ctx, &status);
        if (rc != PB_OK) { report_error(ctx, "status", rc); goto cleanup; }
        if (status.state == 7) { /* Failed: asynchronous diagnostics belong to the snapshot. */
            fputs("Acquisition failed\n", stderr);
            print_snapshot(ctx, stderr);
            goto cleanup;
        }
        wait_ms(5);
    }
    exit_code = 130;
cleanup:
    if (ctx) {
        rc = pb_stop(ctx);
        if (rc != PB_OK) { report_error(ctx, "stop", rc); exit_code = 1; }
        if (consume_pose(&consumer, NULL, max_age_ns) & CONSUMER_FROZEN) {
            ++freezes; puts("FROZEN: stopped");
        }
        if (!print_snapshot(ctx, stdout)) exit_code = 1;
        rc = pb_context_destroy(ctx); /* Consumes ctx even on error. */
        ctx = NULL;
        if (rc != PB_OK) { report_error(NULL, "destroy", rc); exit_code = 1; }
    }
    free(owned_config);
    printf("SUMMARY updates=%" PRIu64 " freezes=%" PRIu64 " references=%" PRIu64 "\n",
           updates, freezes, references);
    return exit_code;
}
