#pragma once

namespace cogip {

namespace app {

/// Initialize LX servomotors
void app_arms_init(void);

/// Left arm folded
void app_left_arm_folded(void);
/// Left arm gripping
void app_left_arm_gripping(void);
/// Left arm giving
void app_left_arm_giving(void);

/// Right arm folded
void app_right_arm_folded(void);
/// Right arm gripping
void app_right_arm_gripping(void);
/// Right arm giving
void app_right_arm_giving(void);

/// Central arm folded
void app_central_arm_folded(void);
/// Central arm gripping prepare
void app_central_arm_gripping_prepare(void);
/// Central arm gripping
void app_central_arm_gripping(void);
/// Central arm taking from right arm
void app_central_arm_taking_right(void);
/// Central arm taking from left arm
void app_central_arm_taking_left(void);
/// Central arm giving to storage wheel
void app_central_arm_giving_wheel(void);

void app_central_arm_gripping_statuette(void);
void app_central_arm_gripping_statuette_up(void);
void app_central_arm_releasing_statuette(void);

void app_central_arm_gripping_replica(void);
void app_central_arm_gripping_replica_up(void);
void app_central_arm_releasing_replica(void);

} // namespace app

} // namespace cogip
