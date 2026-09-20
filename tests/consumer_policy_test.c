#include "../examples/pose_consumer_policy.h"
#include <assert.h>
#include <stdio.h>

int main(void) {
    const uint64_t limit = UINT64_C(100000000);
    ConsumerState state = {0};
    PbPose pose = {0};
    pose.instance_id = 1; pose.session_id = 2; pose.reference_epoch = 3;
    pose.sequence = 20; pose.fresh = 1; pose.quaternion_xyzw[3] = 1;
    assert(consume_pose(&state, NULL, limit) == 0 && !state.has_identity);
    assert(consume_pose(&state, &pose, limit) == (CONSUMER_REFERENCE | CONSUMER_RESUMED | CONSUMER_POSE));
    assert(consume_pose(&state, &pose, limit) == 0);
    pose.age_ns = limit - 1;
    assert(consume_pose(&state, &pose, limit) == 0 && state.active);
    pose.age_ns = limit; /* Still fresh according to the library's 500 ms threshold. */
    assert(consume_pose(&state, &pose, limit) == CONSUMER_FROZEN && !state.active);
    assert(state.sequence == 20 && state.presented_quaternion[3] == 1);
    assert(consume_pose(&state, &pose, limit) == 0);
    pose.age_ns = 0; pose.sequence = 19;
    assert(consume_pose(&state, &pose, limit) == 0 && !state.active);
    pose.sequence = 21;
    assert(consume_pose(&state, &pose, limit) == (CONSUMER_RESUMED | CONSUMER_POSE));
    pose.sequence = 22; pose.fresh = 0;
    assert(consume_pose(&state, &pose, limit) == CONSUMER_FROZEN);
    pose.fresh = 1;
    assert(consume_pose(&state, &pose, limit) == (CONSUMER_RESUMED | CONSUMER_POSE));
    /* Each identity/reference change establishes a new sequence baseline. */
    pose.reference_epoch++; pose.sequence = 1;
    assert(consume_pose(&state, &pose, limit) == (CONSUMER_REFERENCE | CONSUMER_POSE));
    pose.session_id++; pose.sequence = 1;
    assert(consume_pose(&state, &pose, limit) == (CONSUMER_REFERENCE | CONSUMER_POSE));
    pose.instance_id++; pose.reference_epoch = 1;
    assert(consume_pose(&state, &pose, limit) == (CONSUMER_REFERENCE | CONSUMER_POSE));
    assert(consume_pose(&state, NULL, limit) == CONSUMER_FROZEN);
    assert(state.presented_quaternion[3] == 1);
    puts("C11 consumer policy: PASS");
    return 0;
}
