// RIOT includes
#include "log.h"
#include "periph/qdec.h"
#include <inttypes.h>

// Project includes
#include "app.hpp"
#include "app_conf.hpp"
#include "board.h"
#include "drive_controller/DifferentialDriveController.hpp"
#include "drive_controller/DifferentialDriveControllerParameters.hpp"
#include "dualpid_meta_controller/DualPIDMetaController.hpp"
#include "encoder/EncoderQDEC.hpp"
#include "feedforward_combiner_controller/FeedforwardCombinerController.hpp"
#include "localization/LocalizationDifferential.hpp"
#include "motion_control.hpp"
#include "motion_control_common/MetaController.hpp"
#include "motion_control_common/ThrottledController.hpp"
#include "motion_motors_params.hpp"
#include "motor/MotorDriverDRV8873.hpp"
#include "motor/MotorRIOT.hpp"
#include "passthrough_pose_pid_controller/PassthroughPosePIDController.hpp"
#include "passthrough_pose_pid_controller/PassthroughPosePIDControllerIOKeysDefault.hpp"
#include "passthrough_pose_pid_controller/PassthroughPosePIDControllerParameters.hpp"
#include "path/Pose.hpp"
#include "platform.hpp"
#include "platform_engine/PlatformEngine.hpp"
#include "polar_parallel_meta_controller/PolarParallelMetaController.hpp"
#include "pose_pid_controller/PosePIDController.hpp"
#include "pose_pid_controller/PosePIDControllerIOKeysDefault.hpp"
#include "pose_pid_controller/PosePIDControllerParameters.hpp"
#include "pose_straight_filter/PoseStraightFilter.hpp"
#include "pose_straight_filter/PoseStraightFilterIOKeysDefault.hpp"
#include "pose_straight_filter/PoseStraightFilterParameters.hpp"
#include "profile_feedforward_controller/ProfileFeedforwardController.hpp"
#include "quadpid_meta_controller/QuadPIDMetaController.hpp"
#include "speed_filter/SpeedFilter.hpp"
#include "speed_filter/SpeedFilterIOKeysDefault.hpp"
#include "speed_filter/SpeedFilterParameters.hpp"
#include "speed_pid_controller/SpeedPIDController.hpp"
#include "speed_pid_controller/SpeedPIDControllerIOKeysDefault.hpp"
#include "speed_pid_controller/SpeedPIDControllerParameters.hpp"
#include "target_change_detector/TargetChangeDetector.hpp"

#include "quadpid_chain.hpp"
#include "quadpid_feedforward_chain.hpp"

#include "PB_Controller.hpp"
#include "PB_PathPose.hpp"
#include "PB_Pid.hpp"
#include "PB_State.hpp"
#include "etl/limits.h"
#include "telemetry/Telemetry.hpp"

namespace cogip {

namespace pf {

namespace motion_control {

using cogip::utils::operator"" _key_hash;

// Current controller
static uint32_t current_controller_id = 0;

// Protobuf
PB_Pose pb_pose;
PB_Controller pb_controller;
PB_State pb_state;
PB_Pid pb_pid;
PB_Pid_Id pb_pid_id;

// PID tuning period
constexpr uint16_t motion_control_pid_tuning_period_ms = 1500;

// Motion controllers
static cogip::motion_control::QuadPIDMetaController* pf_quadpid_meta_controller;
// Target pose
static cogip::path::Pose target_pose;
// Target speed
static cogip::cogip_defs::Polar target_speed;

// Linear pose PID controller (extern declared in quadpid_chain.hpp)
cogip::pid::PID linear_pose_pid(linear_pose_pid_kp, linear_pose_pid_ki, linear_pose_pid_kd,
                                linear_pose_pid_integral_limit);
// Linear speed PID controller (extern declared in quadpid_chain.hpp)
cogip::pid::PID linear_speed_pid(linear_speed_pid_kp, linear_speed_pid_ki, linear_speed_pid_kd,
                                 linear_speed_pid_integral_limit);
// Angular pose PID controller (extern declared in quadpid_chain.hpp)
cogip::pid::PID angular_pose_pid(angular_pose_pid_kp, angular_pose_pid_ki, angular_pose_pid_kd,
                                 angular_pose_pid_integral_limit);
// Angular speed PID controller (extern declared in quadpid_chain.hpp)
cogip::pid::PID angular_speed_pid(angular_speed_pid_kp, angular_speed_pid_ki, angular_speed_pid_kd,
                                  angular_speed_pid_integral_limit);
// PID null
static cogip::pid::PID null_pid(0, 0, 0, 0);

static void reset_speed_pids()
{
    angular_speed_pid.reset();
    linear_speed_pid.reset();
}

// ============================================================================
// Feedforward chain controllers
// ============================================================================

/// Quad PID Feedforward meta controller.
static cogip::motion_control::QuadPIDMetaController quadpid_feedforward_meta_controller;

/// Linear ProfileFeedforwardController IO keys
static cogip::motion_control::ProfileFeedforwardControllerIOKeys
    linear_profile_feedforward_io_keys = {.pose_error = "linear_pose_error",
                                          .current_speed = "linear_current_speed",
                                          .recompute_profile = "linear_recompute_profile",
                                          .invalidate_profile = "linear_invalidate_profile",
                                          .feedforward_velocity = "linear_feedforward_velocity",
                                          .tracking_error = "linear_tracking_error"};

/// Linear ProfileFeedforwardController parameters
static cogip::motion_control::ProfileFeedforwardControllerParameters
    linear_profile_feedforward_parameters(platform_max_speed_linear_mm_per_period, // max_speed
                                          platform_max_acc_linear_mm_per_period2,  // acceleration
                                          platform_max_dec_linear_mm_per_period2,  // deceleration
                                          true // must_stop_at_end
    );

/// Linear ProfileFeedforwardController
static cogip::motion_control::ProfileFeedforwardController
    linear_profile_feedforward_controller(linear_profile_feedforward_io_keys,
                                          linear_profile_feedforward_parameters);

/// Linear FeedforwardCombinerController IO keys
/// State gating: only active during MOVE_TO_POSITION state (value 1)
static cogip::motion_control::FeedforwardCombinerControllerIOKeys
    linear_feedforward_combiner_io_keys = {.feedforward_velocity = "linear_feedforward_velocity",
                                           .feedback_correction = "linear_feedback_correction",
                                           .speed_order = "linear_speed_order",
                                           .current_state = "pose_straight_filter_state",
                                           .active_state = 1};

/// Linear FeedforwardCombinerController parameters
static cogip::motion_control::FeedforwardCombinerControllerParameters
    linear_feedforward_combiner_parameters;

/// Linear FeedforwardCombinerController
static cogip::motion_control::FeedforwardCombinerController
    linear_feedforward_combiner_controller(linear_feedforward_combiner_io_keys,
                                           linear_feedforward_combiner_parameters);

/// Linear DualPIDMetaController for feedforward chain
static cogip::motion_control::DualPIDMetaController linear_feedforward_dualpid_meta_controller;

/// Linear PosePIDController for tracking error correction (feedforward chain)
/// Uses same parameters as normal pose controller but different IO keys
/// State gating: only active during MOVE_TO_POSITION state (value 1)
static cogip::motion_control::PosePIDControllerIOKeys linear_feedforward_pose_pid_io_keys = {
    .position_error = "linear_tracking_error",
    .current_speed = "linear_current_speed",
    .target_speed = "dummy_target_speed",
    .disable_filter = "dummy_disable",
    .pose_reached = "dummy_pose_reached",
    .speed_order = "linear_feedback_correction",
    .current_state = "pose_straight_filter_state",
    .active_state = 1};

static cogip::motion_control::PosePIDController
    linear_feedforward_pose_controller(linear_feedforward_pose_pid_io_keys,
                                       quadpid_chain::linear_pose_controller_parameters);

/// Angular ProfileFeedforwardController IO keys
static cogip::motion_control::ProfileFeedforwardControllerIOKeys
    angular_profile_feedforward_io_keys = {.pose_error = "angular_pose_error",
                                           .current_speed = "angular_current_speed",
                                           .recompute_profile = "angular_recompute_profile",
                                           .invalidate_profile = "angular_invalidate_profile",
                                           .feedforward_velocity = "angular_feedforward_velocity",
                                           .tracking_error = "angular_tracking_error"};

/// Angular ProfileFeedforwardController parameters
static cogip::motion_control::ProfileFeedforwardControllerParameters
    angular_profile_feedforward_parameters(
        platform_max_speed_angular_deg_per_period, // max_speed
        platform_max_acc_angular_deg_per_period2,  // acceleration
        platform_max_dec_angular_deg_per_period2,  // deceleration (same as acc for angular)
        true                                       // must_stop_at_end
    );

/// Angular ProfileFeedforwardController
static cogip::motion_control::ProfileFeedforwardController
    angular_profile_feedforward_controller(angular_profile_feedforward_io_keys,
                                           angular_profile_feedforward_parameters);

/// Angular FeedforwardCombinerController IO keys
static cogip::motion_control::FeedforwardCombinerControllerIOKeys
    angular_feedforward_combiner_io_keys = {.feedforward_velocity = "angular_feedforward_velocity",
                                            .feedback_correction = "angular_feedback_correction",
                                            .speed_order = "angular_speed_order"};

/// Angular FeedforwardCombinerController parameters
static cogip::motion_control::FeedforwardCombinerControllerParameters
    angular_feedforward_combiner_parameters;

/// Angular FeedforwardCombinerController
static cogip::motion_control::FeedforwardCombinerController
    angular_feedforward_combiner_controller(angular_feedforward_combiner_io_keys,
                                            angular_feedforward_combiner_parameters);

/// Angular DualPIDMetaController for feedforward chain
static cogip::motion_control::DualPIDMetaController angular_feedforward_dualpid_meta_controller;

/// Angular PosePIDController for feedforward chain (tracking error correction)
/// Uses angular_tracking_error which is:
/// - During rotations (profile active): difference between profile position and actual position
/// - During MOVE_TO_POSITION (profile invalidated): equals angular_pose_error for heading
/// maintenance This ensures the trapezoidal velocity profile controls acceleration/deceleration
/// during rotations, while still maintaining heading during linear motion.
static cogip::motion_control::PosePIDControllerIOKeys angular_feedforward_pose_pid_io_keys = {
    .position_error = "angular_tracking_error",
    .current_speed = "angular_current_speed",
    .target_speed = "dummy_target_speed",
    .disable_filter = "dummy_disable",
    .pose_reached = "dummy_pose_reached",
    .speed_order = "angular_feedback_correction"};

static cogip::motion_control::PosePIDController
    angular_feedforward_pose_controller(angular_feedforward_pose_pid_io_keys,
                                        quadpid_chain::angular_pose_controller_parameters);

/// Linear SpeedPIDController for feedforward chain (dedicated instance)
static cogip::motion_control::SpeedPIDController linear_feedforward_speed_controller(
    cogip::motion_control::linear_speed_pid_controller_io_keys_default,
    quadpid_chain::linear_speed_controller_parameters);

/// Angular SpeedPIDController for feedforward chain (dedicated instance)
static cogip::motion_control::SpeedPIDController angular_feedforward_speed_controller(
    cogip::motion_control::angular_speed_pid_controller_io_keys_default,
    quadpid_chain::angular_speed_controller_parameters);

/// PolarParallelMetaController for feedforward chain
static cogip::motion_control::PolarParallelMetaController
    polar_parallel_feedforward_meta_controller;

/// Encoders
static cogip::encoder::EncoderQDEC left_encoder(MOTOR_LEFT, COGIP_BOARD_ENCODER_MODE,
                                                encoder_wheels_resolution_pulses.get());
static cogip::encoder::EncoderQDEC right_encoder(MOTOR_RIGHT, COGIP_BOARD_ENCODER_MODE,
                                                 encoder_wheels_resolution_pulses.get());

/// Odometry
static cogip::localization::LocalizationDifferentialParameters
    localization_params(left_encoder_wheels_diameter_mm, right_encoder_wheels_diameter_mm,
                        encoder_wheels_distance_mm, qdec_left_polarity, qdec_right_polarity);
static cogip::localization::LocalizationDifferential localization(localization_params, left_encoder,
                                                                  right_encoder);

/// Motor driver
static cogip::motor::MotorDriverDRV8873 motor_driver(motion_motors_params);

/// Motors
static cogip::motor::MotorRIOT left_motor(motor_driver, MOTOR_LEFT);
static cogip::motor::MotorRIOT right_motor(motor_driver, MOTOR_RIGHT);

static cogip::drive_controller::DifferentialDriveControllerParameters
    drive_controller_params(motor_wheels_diameter_mm, motor_wheels_diameter_mm,
                            motor_wheels_distance_mm, left_motor_constant, right_motor_constant,
                            min_motor_speed_percent, max_motor_speed_percent,
                            motion_control_thread_period_ms);

static cogip::drive_controller::DifferentialDriveController
    drive_controller(drive_controller_params, left_motor, right_motor);
static void pf_pose_reached_cb(const cogip::motion_control::target_pose_status_t state);
// Motion control engine
static cogip::motion_control::PlatformEngine pf_motion_control_platform_engine(
    localization, drive_controller,
    cogip::motion_control::pose_reached_cb_t::create<pf_pose_reached_cb>(),
    motion_control_thread_period_ms);

/// Initialize platform QuadPID meta controller
/// Return initialized QuadPID meta controller
static cogip::motion_control::QuadPIDMetaController* pf_quadpid_meta_controller_init(void)
{
    // Linear pose loop meta controller (pose controller only, executed at reduced frequency)
    quadpid_chain::linear_pose_loop_meta_controller.add_controller(
        &quadpid_chain::linear_pose_controller);

    // Linear speed loop meta controller (speed filter + anti-blocking + speed controller)
    quadpid_chain::linear_speed_loop_meta_controller.add_controller(
        &quadpid_chain::linear_speed_filter);
    quadpid_chain::linear_speed_loop_meta_controller.add_controller(
        &quadpid_chain::linear_anti_blocking_controller);
    quadpid_chain::linear_speed_loop_meta_controller.add_controller(
        &quadpid_chain::linear_speed_controller);

    // Angular pose loop meta controller (pose controller only, executed at reduced frequency)
    quadpid_chain::angular_pose_loop_meta_controller.add_controller(
        &quadpid_chain::angular_pose_controller);

    // Angular speed loop meta controller (speed filter + anti-blocking + speed controller)
    quadpid_chain::angular_speed_loop_meta_controller.add_controller(
        &quadpid_chain::angular_speed_filter);
    quadpid_chain::angular_speed_loop_meta_controller.add_controller(
        &quadpid_chain::angular_anti_blocking_controller);
    quadpid_chain::angular_speed_loop_meta_controller.add_controller(
        &quadpid_chain::angular_speed_controller);

    // Pose loop PolarParallelMetaController (pose controllers only)
    // --> Linear pose loop meta controller
    // `-> Angular pose loop meta controller
    quadpid_chain::pose_loop_polar_parallel_meta_controller.add_controller(
        &quadpid_chain::linear_pose_loop_meta_controller);
    quadpid_chain::pose_loop_polar_parallel_meta_controller.add_controller(
        &quadpid_chain::angular_pose_loop_meta_controller);

    // Pose loop meta controller (pose_straight_filter + pose loop polar parallel)
    // PoseStraightFilter -> Pose loop PolarParallelMetaController
    quadpid_chain::pose_loop_meta_controller.add_controller(&quadpid_chain::pose_straight_filter);
    quadpid_chain::pose_loop_meta_controller.add_controller(
        &quadpid_chain::pose_loop_polar_parallel_meta_controller);

    // Speed loop PolarParallelMetaController (speed controllers only)
    // --> Linear speed loop meta controller
    // `-> Angular speed loop meta controller
    quadpid_chain::speed_loop_polar_parallel_meta_controller.add_controller(
        &quadpid_chain::linear_speed_loop_meta_controller);
    quadpid_chain::speed_loop_polar_parallel_meta_controller.add_controller(
        &quadpid_chain::angular_speed_loop_meta_controller);

    // QuadPIDMetaController:
    // ThrottledController(pose_loop_meta_controller, 10) -> Speed loop PolarParallelMetaController
    quadpid_chain::quadpid_meta_controller.add_controller(
        &quadpid_chain::throttled_pose_loop_controllers);
    quadpid_chain::quadpid_meta_controller.add_controller(
        &quadpid_chain::speed_loop_polar_parallel_meta_controller);

    return &quadpid_chain::quadpid_meta_controller;
}

/// Restore platform QuadPID meta controller to its original configuration.
static void pf_quadpid_meta_controller_restore(void)
{
    // Linear speed limits
    quadpid_chain::linear_speed_filter_parameters.set_max_speed(
        platform_max_speed_linear_mm_per_period);
    quadpid_chain::linear_speed_filter_parameters.set_max_acceleration(
        platform_max_acc_linear_mm_per_period2);
    // Angular speed limits
    quadpid_chain::angular_speed_filter_parameters.set_max_speed(
        platform_max_speed_angular_deg_per_period);
    quadpid_chain::angular_speed_filter_parameters.set_max_acceleration(
        platform_max_acc_angular_deg_per_period2);

    // Linear pose loop meta controller
    quadpid_chain::linear_pose_loop_meta_controller.replace_controller(
        0, &quadpid_chain::linear_pose_controller);
    // Angular pose loop meta controller
    quadpid_chain::angular_pose_loop_meta_controller.replace_controller(
        0, &quadpid_chain::angular_pose_controller);

    // Linear speed PID controller parameters
    quadpid_chain::linear_speed_controller_parameters.set_pid(&linear_speed_pid);
    // Angular speed PID controller parameters
    quadpid_chain::angular_speed_controller_parameters.set_pid(&angular_speed_pid);

    // Sign target speed according to pose error
    quadpid_chain::passthrough_linear_pose_controller_parameters.set_signed_target_speed(true);
    quadpid_chain::passthrough_angular_pose_controller_parameters.set_signed_target_speed(true);
}

/// Initialize platform QuadPID Feedforward meta controller
/// Return initialized QuadPID Feedforward meta controller
static cogip::motion_control::QuadPIDMetaController*
pf_quadpid_feedforward_meta_controller_init(void)
{
    // Linear feedforward chain:
    //  ProfileFeedforwardController -> PosePIDController -> FeedforwardCombinerController ->
    //  SpeedPIDController
    linear_feedforward_dualpid_meta_controller.add_controller(
        &linear_profile_feedforward_controller);
    linear_feedforward_dualpid_meta_controller.add_controller(&linear_feedforward_pose_controller);
    linear_feedforward_dualpid_meta_controller.add_controller(
        &linear_feedforward_combiner_controller);
    linear_feedforward_dualpid_meta_controller.add_controller(&linear_feedforward_speed_controller);

    // Angular feedforward chain:
    //  ProfileFeedforwardController -> PosePIDController -> FeedforwardCombinerController ->
    //  SpeedPIDController
    angular_feedforward_dualpid_meta_controller.add_controller(
        &angular_profile_feedforward_controller);
    angular_feedforward_dualpid_meta_controller.add_controller(
        &angular_feedforward_pose_controller);
    angular_feedforward_dualpid_meta_controller.add_controller(
        &angular_feedforward_combiner_controller);
    angular_feedforward_dualpid_meta_controller.add_controller(
        &angular_feedforward_speed_controller);

    // PolarParallelMetaController:
    // --> Linear feedforward chain
    // `-> Angular feedforward chain
    polar_parallel_feedforward_meta_controller.add_controller(
        &linear_feedforward_dualpid_meta_controller);
    polar_parallel_feedforward_meta_controller.add_controller(
        &angular_feedforward_dualpid_meta_controller);

    // QuadPIDFeedforwardMetaController:
    // PoseStraightFilter -> PolarParallelFeedforwardMetaController
    // Note: PoseStraightFilter now handles profile recompute signals internally
    // (linear_recompute_profile on MOVE_TO_POSITION entry, angular_recompute_profile on target
    // change and ROTATE_TO_FINAL_ANGLE entry)
    quadpid_feedforward_meta_controller.add_controller(&feedforward_chain::pose_straight_filter);
    quadpid_feedforward_meta_controller.add_controller(&polar_parallel_feedforward_meta_controller);

    return &quadpid_feedforward_meta_controller;
}

/// Restore platform QuadPID Feedforward meta controller to its original
/// configuration.
static void pf_quadpid_feedforward_meta_controller_restore(void)
{
    // Linear speed limits
    quadpid_chain::linear_speed_filter_parameters.set_max_speed(
        platform_max_speed_linear_mm_per_period);
    quadpid_chain::linear_speed_filter_parameters.set_max_acceleration(
        platform_max_acc_linear_mm_per_period2);

    // Angular speed limits
    quadpid_chain::angular_speed_filter_parameters.set_max_speed(
        platform_max_speed_angular_deg_per_period);
    quadpid_chain::angular_speed_filter_parameters.set_max_acceleration(
        platform_max_acc_angular_deg_per_period2);

    // Linear feedforward profile parameters
    linear_profile_feedforward_parameters.set_max_speed(platform_max_speed_linear_mm_per_period);
    linear_profile_feedforward_parameters.set_acceleration(platform_max_acc_linear_mm_per_period2);
    linear_profile_feedforward_parameters.set_deceleration(platform_max_dec_linear_mm_per_period2);

    // Angular feedforward profile parameters
    angular_profile_feedforward_parameters.set_max_speed(platform_max_speed_angular_deg_per_period);
    angular_profile_feedforward_parameters.set_acceleration(
        platform_max_acc_angular_deg_per_period2);
    angular_profile_feedforward_parameters.set_deceleration(
        platform_max_dec_angular_deg_per_period2);

    // Linear speed PID controller parameters
    quadpid_chain::linear_speed_controller_parameters.set_pid(&linear_speed_pid);
    // Angular speed PID controller parameters
    quadpid_chain::angular_speed_controller_parameters.set_pid(&angular_speed_pid);
}

/// Disable linear pose control to avoid stopping once point is reached.
static void pf_quadpid_meta_controller_linear_pose_controller_disabled(void)
{
    // Restore quad PID controller to its original state
    pf_quadpid_meta_controller_restore();

    // Disable pose PID correction by using a passthrough controller
    quadpid_chain::linear_pose_loop_meta_controller.replace_controller(
        0, &quadpid_chain::passthrough_linear_pose_controller);
}

/// Disable angular correction and linear speed filtering for linear speed PID
/// setup.
static void pf_quadpid_meta_controller_linear_speed_controller_test_setup(void)
{
    // Restore quad PID controller to its original state
    pf_quadpid_meta_controller_restore();

    // Disable speed filter by setting maximum speed and acceleration to
    // unreachable limits
    quadpid_chain::linear_speed_filter_parameters.set_max_speed(
        etl::numeric_limits<uint16_t>::max());
    quadpid_chain::linear_speed_filter_parameters.set_max_acceleration(
        etl::numeric_limits<uint16_t>::max());

    // Disable pose PID correction by using a passthrough controller
    quadpid_chain::linear_pose_loop_meta_controller.replace_controller(
        0, &quadpid_chain::passthrough_linear_pose_controller);

    // Disable angular speed loop by using a PID with all gain set to zero
    quadpid_chain::angular_speed_controller_parameters.set_pid(&null_pid);

    // Do not sign target speed
    quadpid_chain::passthrough_linear_pose_controller_parameters.set_signed_target_speed(false);
}

/// Disable linear correction and angular speed filtering for angular speed PID
/// setup.
static void pf_quadpid_meta_controller_angular_speed_controller_test_setup(void)
{
    // Restore quad PID controller to its original state
    pf_quadpid_meta_controller_restore();

    // Disable speed filter by setting maximum speed and acceleration to
    // unreachable limits
    quadpid_chain::angular_speed_filter_parameters.set_max_speed(
        etl::numeric_limits<uint16_t>::max());
    quadpid_chain::angular_speed_filter_parameters.set_max_acceleration(
        etl::numeric_limits<uint16_t>::max());

    // Disable pose PID correction by using a passthrough controller
    quadpid_chain::angular_pose_loop_meta_controller.replace_controller(
        0, &quadpid_chain::passthrough_angular_pose_controller);

    // Disable linear speed loop by using a PID with all gain set to zero
    quadpid_chain::linear_speed_controller_parameters.set_pid(&null_pid);

    // Do not sign target speed
    quadpid_chain::passthrough_angular_pose_controller_parameters.set_signed_target_speed(false);
}

/// Update current speed from quadrature encoders measure.
static void pf_encoder_reset(void)
{
    left_encoder.reset();
    right_encoder.reset();
}

static void pf_send_pid(PB_PidEnum id)
{
    pb_pid.clear();
    pb_pid.set_id(id);

    switch (id) {
    case PB_PidEnum::LINEAR_POSE_PID:
        linear_pose_pid.pb_copy(pb_pid);
        break;
    case PB_PidEnum::ANGULAR_POSE_PID:
        angular_pose_pid.pb_copy(pb_pid);
        break;
    case PB_PidEnum::LINEAR_SPEED_PID:
        linear_speed_pid.pb_copy(pb_pid);
        break;
    case PB_PidEnum::ANGULAR_SPEED_PID:
        angular_speed_pid.pb_copy(pb_pid);
        break;
    }

    pf_get_canpb().send_message(pid_uuid, &pb_pid);
}

/// Handle pid request command message.
static void _handle_pid_request(cogip::canpb::ReadBuffer& buffer)
{
    pb_pid_id.clear();
    EmbeddedProto::Error error = pb_pid_id.deserialize(buffer);
    if (error != EmbeddedProto::Error::NO_ERRORS) {
        LOG_ERROR("Pid request: Protobuf deserialization error: %d\n", static_cast<int>(error));
        return;
    } else {
        LOG_INFO("Pid request: %" PRIu32 "\n", static_cast<uint32_t>(pb_pid_id.id()));
    }

    // Send PIDs
    pf_send_pid(pb_pid_id.id());
}

/// Handle new pid config message.
static void _handle_new_pid_config(cogip::canpb::ReadBuffer& buffer)
{
    pb_pid.clear();

    EmbeddedProto::Error error = pb_pid.deserialize(buffer);
    if (error != EmbeddedProto::Error::NO_ERRORS) {
        LOG_ERROR("New pid config: Protobuf deserialization error: %d\n", static_cast<int>(error));
        return;
    }

    switch (pb_pid.id()) {
    case PB_PidEnum::LINEAR_POSE_PID:
        linear_pose_pid.pb_read(pb_pid);
        break;
    case PB_PidEnum::ANGULAR_POSE_PID:
        angular_pose_pid.pb_read(pb_pid);
        break;
    case PB_PidEnum::LINEAR_SPEED_PID:
        linear_speed_pid.pb_read(pb_pid);
        break;
    case PB_PidEnum::ANGULAR_SPEED_PID:
        angular_speed_pid.pb_read(pb_pid);
        break;
    }
}

/// Handle controller change request
static void _handle_set_controller(cogip::canpb::ReadBuffer& buffer)
{
    pb_controller.clear();

    EmbeddedProto::Error error = pb_controller.deserialize(buffer);
    if (error != EmbeddedProto::Error::NO_ERRORS) {
        LOG_ERROR("Controller change request: Protobuf deserialization error: %d\n",
                  static_cast<int>(error));
        return;
    }

    // Change controller
    current_controller_id = static_cast<uint32_t>(pb_controller.id());
    switch (static_cast<uint32_t>(pb_controller.id())) {
    case static_cast<uint32_t>(PB_ControllerEnum::LINEAR_SPEED_TEST):
        LOG_INFO("Change to controller: LINEAR_SPEED_TEST\n");
        pf_quadpid_meta_controller_linear_speed_controller_test_setup();
        pf_motion_control_platform_engine.set_timeout_enable(true);
        break;

    case static_cast<uint32_t>(PB_ControllerEnum::ANGULAR_SPEED_TEST):
        LOG_INFO("Change to controller: ANGULAR_SPEED_TEST\n");
        pf_quadpid_meta_controller_angular_speed_controller_test_setup();
        pf_motion_control_platform_engine.set_timeout_enable(true);
        break;

    case static_cast<uint32_t>(PB_ControllerEnum::LINEAR_POSE_DISABLED):
        LOG_INFO("Change to controller: LINEAR_POSE_DISABLED\n");
        pf_quadpid_meta_controller_linear_pose_controller_disabled();
        pf_motion_control_platform_engine.set_timeout_enable(false);
        break;

    case static_cast<uint32_t>(PB_ControllerEnum::QUADPID_FEEDFORWARD):
        LOG_INFO("Change to controller: QUADPID_FEEDFORWARD\n");
        pf_quadpid_feedforward_meta_controller_restore();
        pf_motion_control_platform_engine.set_controller(&quadpid_feedforward_meta_controller);
        pf_motion_control_platform_engine.set_timeout_enable(false);
        break;

    case static_cast<uint32_t>(PB_ControllerEnum::QUADPID):
    default:
        LOG_INFO("Change to controller: QUADPID\n");
        pf_quadpid_meta_controller_restore();
        pf_motion_control_platform_engine.set_controller(pf_quadpid_meta_controller);
        pf_motion_control_platform_engine.set_timeout_enable(false);
        break;
    }

    pf_motion_control_platform_engine.set_current_cycle(0);
    pf_motion_control_platform_engine.dump_pipeline();
}

static void pf_pose_reached_cb(const cogip::motion_control::target_pose_status_t state)
{
    // Previous target pose status flag to avoid flooding protobuf can bus.
    static cogip::motion_control::target_pose_status_t previous_target_pose_status =
        cogip::motion_control::target_pose_status_t::moving;

    switch (state) {
    case cogip::motion_control::target_pose_status_t::moving:
        // Do nothing when robot is moving
        break;

    case cogip::motion_control::target_pose_status_t::reached:
        // Send message in case of final pose reached only.
        if (previous_target_pose_status != state) {
            if (pf_motion_control_platform_engine.target_pose().is_intermediate()) {
                pf_get_canpb().send_message(intermediate_pose_reached_uuid);
            } else {
                pf_get_canpb().send_message(pose_reached_uuid);
            }

            // Reset previous speed orders
            quadpid_chain::linear_speed_filter.reset_previous_speed_order();
            quadpid_chain::angular_speed_filter.reset_previous_speed_order();
        }

        break;

    case cogip::motion_control::target_pose_status_t::intermediate_reached:
        // Do nothing on intermediate pose
        break;

    case cogip::motion_control::target_pose_status_t::blocked:
        if (pf_motion_control_platform_engine.target_pose().bypass_anti_blocking()) {
            // Set current robot pose as target pose to avoid robot moving
            target_pose.set_x(pf_motion_control_platform_engine.current_pose().x());
            target_pose.set_y(pf_motion_control_platform_engine.current_pose().y());
            target_pose.set_O(pf_motion_control_platform_engine.current_pose().O());
            pf_motion_control_platform_engine.set_target_pose(target_pose);

            // Consider pose_reached as anti blocking is bypassed
            pf_motion_control_platform_engine.set_pose_reached(
                cogip::motion_control::target_pose_status_t::reached);

            // As pose is reached, pose straight filter state machine is in finished
            // state (reset both chains as only one is active at a time)
            quadpid_chain::pose_straight_filter.force_finished_state();
            feedforward_chain::pose_straight_filter.force_finished_state();

            // Reset previous speed orders
            quadpid_chain::linear_speed_filter.reset_previous_speed_order();
            quadpid_chain::angular_speed_filter.reset_previous_speed_order();

            LOG_WARNING("BLOCKED bypassed\n");

            pf_get_canpb().send_message(pose_reached_uuid);
        } else {
            // Stop motors
            left_motor.set_speed(0);
            right_motor.set_speed(0);

            // As motors are stopped, pose straight filter state machine is in
            // finished state (reset both chains as only one is active at a time)
            quadpid_chain::pose_straight_filter.force_finished_state();
            feedforward_chain::pose_straight_filter.force_finished_state();

            // Reset previous speed orders
            quadpid_chain::linear_speed_filter.reset_previous_speed_order();
            quadpid_chain::angular_speed_filter.reset_previous_speed_order();

            LOG_WARNING("BLOCKED\n");

            if (previous_target_pose_status !=
                cogip::motion_control::target_pose_status_t::blocked) {
                pf_get_canpb().send_message(blocked_uuid);
            }
        }

        break;

    default:
        break;
    }

    // Reset speed PIDs if a pose has been reached (intermediate, blocked or
    // final). Pose PIDs do not need to be reset as they only have Kp (no sum of
    // error).
    if (previous_target_pose_status != state &&
        state != cogip::motion_control::target_pose_status_t::moving) {
        reset_speed_pids();
    }

    // Backup target pose status flag to avoid flooding protobuf serial bus on
    // next cycle.
    previous_target_pose_status = state;
}

/*
 * Used for simulation in motion_motors_params.hpp only if MOTION_MOTORS_POST_CB
 * is defined in board.h this way:
 * #define MOTION_MOTORS_POST_CB cogip_native_motor_driver_qdec_simulation
 */
extern "C" {
extern int32_t qdecs_value[QDEC_NUMOF];
}

void cogip_native_motor_driver_qdec_simulation(const motor_driver_t* motor_driver, uint8_t motor_id,
                                               int32_t pwm_duty_cycle)
{
    // Unused variables
    (void)motor_driver;
    (void)motor_id;
    (void)pwm_duty_cycle;

    float wheels_perimeter = M_PI * left_encoder_wheels_diameter_mm.get();
    float pulse_per_mm = encoder_wheels_resolution_pulses.get() / wheels_perimeter;
    float wheels_distance_pulse = encoder_wheels_distance_mm.get() * pulse_per_mm;
    float pulse_per_degree = (wheels_distance_pulse * 2 * M_PI) / 360;

    // On native architecture set speeds at their theorical value, no error.
    if (pf_motion_control_platform_engine.pose_reached() !=
        cogip::motion_control::target_pose_status_t::reached) {
        // Get speed commands from IO
        float linear_speed_cmd = 0.0f;
        float angular_speed_cmd = 0.0f;
        if (auto opt = pf_motion_control_platform_engine.io().get_as<float>("linear_speed_order")) {
            linear_speed_cmd = *opt;
        }
        if (auto opt =
                pf_motion_control_platform_engine.io().get_as<float>("angular_speed_order")) {
            angular_speed_cmd = *opt;
        }

        qdecs_value[MOTOR_RIGHT] =
            (linear_speed_cmd * pulse_per_mm + angular_speed_cmd * pulse_per_degree / 2) *
            qdec_right_polarity.get();
        qdecs_value[MOTOR_LEFT] =
            (linear_speed_cmd * pulse_per_mm - angular_speed_cmd * pulse_per_degree / 2) *
            qdec_left_polarity.get();
    }
}

void pf_send_pb_pose(void)
{
    cogip_defs::Pose current_pose = pf_motion_control_platform_engine.current_pose();

    // Copy current pose
    current_pose.pb_copy(pb_pose);

    pf_get_canpb().send_message(pose_order_uuid, &pb_pose);
}

void pf_send_pb_state(void)
{
    pb_state.clear();
    pb_state.mutable_cycle() = pf_motion_control_platform_engine.current_cycle();
    pf_motion_control_platform_engine.current_speed().pb_copy(pb_state.mutable_speed_current());
    pf_motion_control_platform_engine.target_speed().pb_copy(pb_state.mutable_speed_order());

    pf_get_canpb().send_message(state_uuid, &pb_state);
}

void pf_send_encoder_telemetry(void)
{
    cogip::telemetry::Telemetry::send<int64_t>("encoder_left"_key_hash, left_encoder.counter());
    cogip::telemetry::Telemetry::send<int64_t>("encoder_right"_key_hash, right_encoder.counter());
}

void pf_handle_brake([[maybe_unused]] cogip::canpb::ReadBuffer& buffer)
{
    pf_motion_control_platform_engine.set_target_speed(cogip::cogip_defs::Polar(0, 0));
    reset_speed_pids();
    // Reset both chains as only one is active at a time
    quadpid_chain::pose_straight_filter.force_finished_state();
    feedforward_chain::pose_straight_filter.force_finished_state();
}

void pf_handle_game_end([[maybe_unused]] cogip::canpb::ReadBuffer& buffer)
{
    pf_motion_control_platform_engine.set_target_speed(cogip::cogip_defs::Polar(0, 0));

    // Reset previous speed orders
    quadpid_chain::angular_speed_filter.reset_previous_speed_order();
    quadpid_chain::linear_speed_filter.reset_previous_speed_order();

    // Reset PIDs
    reset_speed_pids();

    // Force position filter finished state (reset both chains as only one is active at a time)
    quadpid_chain::pose_straight_filter.force_finished_state();
    feedforward_chain::pose_straight_filter.force_finished_state();

    // Disable motion control to avoid new motion
    pf_disable_motion_control();
}

void pf_handle_target_pose(cogip::canpb::ReadBuffer& buffer)
{
    // Retrieve new target pose from protobuf message
    PB_PathPose pb_path_target_pose;
    EmbeddedProto::Error error = pb_path_target_pose.deserialize(buffer);
    if (error != EmbeddedProto::Error::NO_ERRORS) {
        LOG_ERROR("Pose to reach: Protobuf deserialization error: %d\n", static_cast<int>(error));
        return;
    }

    cogip::path::Pose previous_target_pose = target_pose;

    // Target pose
    target_pose.pb_read(pb_path_target_pose);
    LOG_INFO("New target pose: x=%.2f, y=%.2f, O=%.2f\n", static_cast<double>(target_pose.x()),
             static_cast<double>(target_pose.y()), static_cast<double>(target_pose.O()));

    // Target speed
    target_speed.set_distance(
        (platform_max_speed_linear_mm_per_period * target_pose.max_speed_ratio_linear()) / 100);
    target_speed.set_angle(
        (platform_max_speed_angular_deg_per_period * target_pose.max_speed_ratio_angular()) / 100);
    pf_motion_control_platform_engine.set_target_speed(target_speed);
    LOG_INFO("Target speed: linear=%.2f mm/period, angular=%.2f deg/period\n",
             static_cast<double>(target_speed.distance()),
             static_cast<double>(target_speed.angle()));

    // Set final orientation bypassing
    target_pose.bypass_final_orientation()
        ? quadpid_chain::pose_straight_filter_parameters.bypass_final_orientation_on()
        : quadpid_chain::pose_straight_filter_parameters.bypass_final_orientation_off();

    // Set target speed for passthrough controllers
    quadpid_chain::passthrough_linear_pose_controller_parameters.set_target_speed(
        target_speed.distance());
    quadpid_chain::passthrough_angular_pose_controller_parameters.set_target_speed(
        target_speed.angle());

    if (target_pose.timeout_ms()) {
        pf_motion_control_platform_engine.set_timeout_enable(true);
        pf_motion_control_platform_engine.set_timeout_ms(target_pose.timeout_ms());
    } else {
        pf_motion_control_platform_engine.set_timeout_enable(false);
    }

    // Deal with the first pose in the list
    pf_motion_control_platform_engine.set_target_pose(target_pose);

    // New target pose, the robot is moving
    pf_motion_control_platform_engine.set_pose_reached(
        cogip::motion_control::target_pose_status_t::moving);

    pf_motion_control_reset();

    pf_enable_motion_control();
}

void pf_handle_start_pose(cogip::canpb::ReadBuffer& buffer)
{
    PB_PathPose pb_start_pose;
    EmbeddedProto::Error error = pb_start_pose.deserialize(buffer);
    if (error != EmbeddedProto::Error::NO_ERRORS) {
        LOG_ERROR("Current pose: Protobuf deserialization error: %d\n", static_cast<int>(error));
        return;
    }
    cogip::path::Pose start_pose;
    start_pose.pb_read(pb_start_pose);
    cogip::path::Pose pose(start_pose.x(), start_pose.y(), start_pose.O());

    pf_motion_control_platform_engine.set_current_pose(pose);
    pf_motion_control_platform_engine.set_target_pose(pose);
    // New start pose, the robot is not moving
    pf_motion_control_platform_engine.set_pose_reached(
        cogip::motion_control::target_pose_status_t::reached);

    pf_motion_control_platform_engine.set_current_cycle(0);
}

void pf_start_motion_control(void)
{
    // Start engine thread
    pf_motion_control_platform_engine.start_thread();
}

void pf_motion_control_reset(void)
{
    // Reset previous speed orders
    quadpid_chain::angular_speed_filter.reset_previous_speed_order();
    quadpid_chain::linear_speed_filter.reset_previous_speed_order();
    // Reset PIDs
    reset_speed_pids();

    // Reset pose straight filter state (reset both chains as only one is active at a time)
    quadpid_chain::pose_straight_filter.reset_current_state();
    feedforward_chain::pose_straight_filter.reset_current_state();
}

void pf_disable_motion_control()
{
    pf_motion_control_platform_engine.disable();

    // Disable motors
    left_motor.disable();
    right_motor.disable();
}

void pf_enable_motion_control()
{
    pf_motion_control_platform_engine.enable();
}

void pf_init_motion_control(void)
{
    // Init motors
    left_motor.init();
    right_motor.init();

    // Init encoders
    int error = left_encoder.init();
    if (error) {
        LOG_ERROR("QDEC %" PRIu32 " not initialized, error=%d\n", static_cast<uint32_t>(MOTOR_LEFT),
                  error);
    }
    error = right_encoder.init();
    if (error) {
        LOG_ERROR("QDEC %" PRIu32 " not initialized, error=%d\n",
                  static_cast<uint32_t>(MOTOR_RIGHT), error);
    }

    // Init controllers
    pf_quadpid_meta_controller = pf_quadpid_meta_controller_init();
    pf_quadpid_feedforward_meta_controller_init();

    // Associate default controller (QUADPID) to the engine
    pf_motion_control_platform_engine.set_controller(pf_quadpid_meta_controller);
    pf_motion_control_platform_engine.dump_pipeline();

    // Set timeout for speed only loops as no pose has to be reached
    pf_motion_control_platform_engine.set_timeout_ms(motion_control_pid_tuning_period_ms /
                                                     motion_control_thread_period_ms);

    //// Register pid request command
    pf_get_canpb().register_message_handler(
        pid_request_uuid, cogip::canpb::message_handler_t::create<_handle_pid_request>());

    // Register new pids config
    pf_get_canpb().register_message_handler(
        pid_uuid, cogip::canpb::message_handler_t::create<_handle_new_pid_config>());

    // Register new pids config
    pf_get_canpb().register_message_handler(
        controller_uuid, cogip::canpb::message_handler_t::create<_handle_set_controller>());

    pf_encoder_reset();
    pf_disable_motion_control();
}

} // namespace motion_control

} // namespace pf

} // namespace cogip
