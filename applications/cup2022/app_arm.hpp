#pragma once

namespace cogip {
namespace app {
namespace arms {

/// Left arm folded
void left_arm_folded();
/// Left arm gripping
void left_arm_gripping();
/// Left arm giving
void left_arm_giving();

/// Right arm folded
void right_arm_folded();
/// Right arm gripping
void right_arm_gripping();
/// Right arm giving
void right_arm_giving();

/// Central arm folded
void central_arm_folded();
/// Central arm gripping prepare
void central_arm_gripping_prepare();
/// Central arm gripping
void central_arm_gripping();
/// Central arm taking from right arm
void central_arm_taking_right();
/// Central arm taking from left arm
void central_arm_taking_left();
/// Central arm giving to storage wheel
void central_arm_giving_wheel();

/// Central arm drop gallery low prepare
void central_drop_gallery_low_prepare();
/// Central arm drop gallery low release
void central_drop_gallery_low_release();
/// Central arm drop gallery high prepare
void central_drop_gallery_high_prepare();
/// Central arm drop gallery high release
void central_drop_gallery_high_release();

void central_arm_gripping_statuette();
void central_arm_gripping_statuette_up();
void central_arm_releasing_statuette();

void central_arm_gripping_replica();
void central_arm_gripping_replica_up();
void central_arm_releasing_replica();

} // namespace arms
} // namespace app
} // namespace cogip
