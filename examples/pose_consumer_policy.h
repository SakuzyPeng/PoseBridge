#ifndef POSE_CONSUMER_POLICY_H
#define POSE_CONSUMER_POLICY_H

#include "posebridge.h"
#include <string.h>

/* Example host policy, not part of the PoseBridge ABI. Keep on one control thread. */
typedef struct ConsumerState {
    uint64_t instance_id, session_id, reference_epoch, sequence;
    int has_identity, active;
    float presented_quaternion[4];
} ConsumerState;

enum ConsumerEvent {
    CONSUMER_POSE = 1,
    CONSUMER_REFERENCE = 2,
    CONSUMER_FROZEN = 4,
    CONSUMER_RESUMED = 8
};

static inline unsigned consume_pose(ConsumerState *state, const PbPose *pose, uint64_t max_age_ns) {
    /* Check age before deduplication: an unchanged sequence can become stale. */
    if (!pose || !pose->fresh || pose->age_ns >= max_age_ns) {
        const unsigned events = state->active ? CONSUMER_FROZEN : 0;
        state->active = 0;
        return events; /* Preserve the last presented orientation. */
    }
    const int reference_changed = !state->has_identity || state->instance_id != pose->instance_id
        || state->session_id != pose->session_id || state->reference_epoch != pose->reference_epoch;
    if (!reference_changed && pose->sequence <= state->sequence) {
        return 0; /* Duplicate/older samples cannot resume a frozen host. */
    }
    unsigned events = CONSUMER_POSE;
    if (reference_changed) events |= CONSUMER_REFERENCE;
    if (!state->active) events |= CONSUMER_RESUMED;
    state->instance_id = pose->instance_id;
    state->session_id = pose->session_id;
    state->reference_epoch = pose->reference_epoch;
    state->sequence = pose->sequence;
    state->has_identity = 1;
    state->active = 1;
    memcpy(state->presented_quaternion, pose->quaternion_xyzw, sizeof(state->presented_quaternion));
    return events;
}

#endif
