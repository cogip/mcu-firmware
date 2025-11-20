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
#include "quadpid_meta_controller/QuadPIDMetaController.hpp"
#include "speed_filter/SpeedFilter.hpp"
#include "speed_filter/SpeedFilterIOKeysDefault.hpp"
#include "speed_filter/SpeedFilterParameters.hpp"
#include "speed_pid_controller/SpeedPIDController.hpp"
#include "speed_pid_controller/SpeedPIDControllerIOKeysDefault.hpp"
#include "speed_pid_controller/SpeedPIDControllerParameters.hpp"

#include "PB_Controller.hpp"
#include "PB_PathPose.hpp"
#include "PB_Pid.hpp"
#include "PB_State.hpp"
#include "etl/limits.h"

namespace cogip {

namespace pf {

namespace motion_control {

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

// Linear pose PID controller
static cogip::pid::PID linear_pose_pid(linear_pose_pid_kp, linear_pose_pid_ki, linear_pose_pid_kd,
                                       linear_pose_pid_integral_limit);
// Linear speed PID controller
static cogip::pid::PID linear_speed_pid(linear_speed_pid_kp, linear_speed_pid_ki,
                                        linear_speed_pid_kd, linear_speed_pid_integral_limit);
// Angular pose PID controller
static cogip::pid::PID angular_pose_pid(angular_pose_pid_kp, angular_pose_pid_ki,
                                        angular_pose_pid_kd, angular_pose_pid_integral_limit);
// Angular speed PID controller
static cogip::pid::PID angular_speed_pid(angular_speed_pid_kp, angular_speed_pid_ki,
                                         angular_speed_pid_kd, angular_speed_pid_integral_limit);
// PID null
static cogip::pid::PID null_pid(0, 0, 0, 0);

static void reset_speed_pids()
{
    angular_speed_pid.reset();
    linear_speed_pid.reset();
}

/// PoseStraightFilter parameters.
static cogip::motion_control::PoseStraightFilterParameters pose_straight_filter_parameters =
    cogip::motion_control::PoseStraightFilterParameters(
        angular_threshold, linear_threshold, angular_intermediate_threshold,
        platform_max_dec_angular_deg_per_period2, platform_max_dec_linear_mm_per_period2);
/// PoseStraightFilter controller to make the robot always moves in a straight
/// line.
static cogip::motion_control::PoseStraightFilter pose_straight_filter =
    cogip::motion_control::PoseStraightFilter(
        cogip::motion_control::pose_straight_filter_io_keys_default,
        pose_straight_filter_parameters);

/// MetaController for pose loop controllers (executed at reduced frequency)
static cogip::motion_control::MetaController<2> pose_loop_meta_controller;

/// PolarParallelMetaController to split linear and angular chain for pose loop controllers.
static cogip::motion_control::PolarParallelMetaController pose_loop_polar_parallel_meta_controller;

/// PolarParallelMetaController to split linear and angular chain for speed loop controllers.
static cogip::motion_control::PolarParallelMetaController speed_loop_polar_parallel_meta_controller;

/// Linear MetaController for pose loop controller only
static cogip::motion_control::MetaController<1> linear_pose_loop_meta_controller;

/// Linear MetaController for speed loop controllers
static cogip::motion_control::MetaController<2> linear_speed_loop_meta_controller;
/// Linear PosePIDControllerParameters.
static cogip::motion_control::PosePIDControllerParameters
    linear_pose_controller_parameters(&linear_pose_pid);
/// Linear PosePIDController that provides SpeedPIDController order first
/// filtered by SpeedFilter.
static cogip::motion_control::PosePIDController
    linear_pose_controller(cogip::motion_control::linear_pose_pid_controller_io_keys_default,
                           linear_pose_controller_parameters);
/// Linear SpeedFilterParameters.
static cogip::motion_control::SpeedFilterParameters linear_speed_filter_parameters(
    platform_min_speed_linear_mm_per_period, platform_max_speed_linear_mm_per_period,
    platform_max_acc_linear_mm_per_period2, platform_linear_antiblocking,
    platform_linear_anti_blocking_speed_threshold_mm_per_period,
    platform_linear_anti_blocking_error_threshold_mm_per_period,
    platform_linear_anti_blocking_blocked_cycles_nb_threshold);
/// Linear SpeedFilter to limit speed and acceleration for linear
/// SpeedPIDController.
static cogip::motion_control::SpeedFilter
    linear_speed_filter(cogip::motion_control::linear_speed_filter_io_keys_default,
                        linear_speed_filter_parameters);
/// Linear SpeedPIDControllerParameters.
static cogip::motion_control::SpeedPIDControllerParameters
    linear_speed_controller_parameters(&linear_speed_pid);
/// Linear SpeedPIDController to compute linear command to send to motors.
static cogip::motion_control::SpeedPIDController
    linear_speed_controller(cogip::motion_control::linear_speed_pid_controller_io_keys_default,
                            linear_speed_controller_parameters);

/// Angular MetaController for pose loop controller only
static cogip::motion_control::MetaController<1> angular_pose_loop_meta_controller;

/// Angular MetaController for speed loop controllers
static cogip::motion_control::MetaController<2> angular_speed_loop_meta_controller;

/// Angular PosePIDControllerParameters.
static cogip::motion_control::PosePIDControllerParameters
    angular_pose_controller_parameters(&angular_pose_pid);
/// Angular PosePIDController that provides SpeedPIDController order first
/// filtered by SpeedFilter.
static cogip::motion_control::PosePIDController
    angular_pose_controller(cogip::motion_control::angular_pose_pid_controller_io_keys_default,
                            angular_pose_controller_parameters);
/// Angular SpeedFilterParameters.
static cogip::motion_control::SpeedFilterParameters
    angular_speed_filter_parameters(platform_min_speed_angular_deg_per_period,
                                    platform_max_speed_angular_deg_per_period,
                                    platform_max_acc_angular_deg_per_period2);
/// Angular SpeedFilter to limit speed and acceleration for angular
/// SpeedPIDController.
static cogip::motion_control::SpeedFilter
    angular_speed_filter(cogip::motion_control::angular_speed_filter_io_keys_default,
                         angular_speed_filter_parameters);
/// Angular SpeedPIDControllerParameters.
static cogip::motion_control::SpeedPIDControllerParameters
    angular_speed_controller_parameters(&angular_speed_pid);
/// Angular SpeedPIDController to compute angular command to send to motors.
static cogip::motion_control::SpeedPIDController
    angular_speed_controller(cogip::motion_control::angular_speed_pid_controller_io_keys_default,
                             angular_speed_controller_parameters);

/// Throttled controller wrapping all pose loop controllers
static cogip::motion_control::ThrottledController
    throttled_pose_loop_controllers(&pose_loop_meta_controller, pose_controllers_throttle_divider);

/// Linear PassthroughPosePIDControllerParameters.
static cogip::motion_control::PassthroughPosePIDControllerParameters
    passthrough_linear_pose_controller_parameters(platform_max_speed_linear_mm_per_period, true);
/// Linear PassthroughPosePIDController replaces linear PosePIDController to
/// bypass it, imposing target speed as speed order.
static cogip::motion_control::PassthroughPosePIDController passthrough_linear_pose_controller(
    cogip::motion_control::linear_passthrough_pose_pid_controller_io_keys_default,
    passthrough_linear_pose_controller_parameters);
/// Angular PassthroughPosePIDControllerParameters.
static cogip::motion_control::PassthroughPosePIDControllerParameters
    passthrough_angular_pose_controller_parameters(platform_max_speed_angular_deg_per_period, true);
/// Angular PassthroughPosePIDController replaces angular PosePIDController to
/// bypass it, imposing target speed as speed order.
static cogip::motion_control::PassthroughPosePIDController passthrough_angular_pose_controller(
    cogip::motion_control::angular_passthrough_pose_pid_controller_io_keys_default,
    passthrough_angular_pose_controller_parameters);

/// Quad PID meta controller.
static cogip::motion_control::QuadPIDMetaController quadpid_meta_controller;

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
    linear_pose_loop_meta_controller.add_controller(&linear_pose_controller);

    // Linear speed loop meta controller (speed filter + speed controller, executed every cycle)
    linear_speed_loop_meta_controller.add_controller(&linear_speed_filter);
    linear_speed_loop_meta_controller.add_controller(&linear_speed_controller);

    // Angular pose loop meta controller (pose controller only, executed at reduced frequency)
    angular_pose_loop_meta_controller.add_controller(&angular_pose_controller);

    // Angular speed loop meta controller (speed filter + speed controller, executed every cycle)
    angular_speed_loop_meta_controller.add_controller(&angular_speed_filter);
    angular_speed_loop_meta_controller.add_controller(&angular_speed_controller);

    // Pose loop PolarParallelMetaController (pose controllers only)
    // --> Linear pose loop meta controller
    // `-> Angular pose loop meta controller
    pose_loop_polar_parallel_meta_controller.add_controller(&linear_pose_loop_meta_controller);
    pose_loop_polar_parallel_meta_controller.add_controller(&angular_pose_loop_meta_controller);

    // Pose loop meta controller (pose_straight_filter + pose loop polar parallel)
    // PoseStraightFilter -> Pose loop PolarParallelMetaController
    pose_loop_meta_controller.add_controller(&pose_straight_filter);
    pose_loop_meta_controller.add_controller(&pose_loop_polar_parallel_meta_controller);

    // Speed loop PolarParallelMetaController (speed controllers only)
    // --> Linear speed loop meta controller
    // `-> Angular speed loop meta controller
    speed_loop_polar_parallel_meta_controller.add_controller(&linear_speed_loop_meta_controller);
    speed_loop_polar_parallel_meta_controller.add_controller(&angular_speed_loop_meta_controller);

    // QuadPIDMetaController:
    // ThrottledController(pose_loop_meta_controller, 10) -> Speed loop PolarParallelMetaController
    quadpid_meta_controller.add_controller(&throttled_pose_loop_controllers);
    quadpid_meta_controller.add_controller(&speed_loop_polar_parallel_meta_controller);

    return &quadpid_meta_controller;
}

/// Restore platform QuadPID meta controller to its original configuration.
static void pf_quadpid_meta_controller_restore(void)
{
    // Linear speed limits
    linear_speed_filter_parameters.set_max_speed(platform_max_speed_linear_mm_per_period);
    linear_speed_filter_parameters.set_max_acceleration(platform_max_acc_linear_mm_per_period2);
    // Angular speed limits
    angular_speed_filter_parameters.set_max_speed(platform_max_speed_angular_deg_per_period);
    angular_speed_filter_parameters.set_max_acceleration(platform_max_acc_angular_deg_per_period2);

    // Linear pose loop meta controller
    linear_pose_loop_meta_controller.replace_controller(0, &linear_pose_controller);
    // Angular pose loop meta controller
    angular_pose_loop_meta_controller.replace_controller(0, &angular_pose_controller);

    // Linear speed PID controller parameters
    linear_speed_controller_parameters.set_pid(&linear_speed_pid);
    // Angular speed PID controller parameters
    angular_speed_controller_parameters.set_pid(&angular_speed_pid);

    // Sign target speed according to pose error
    passthrough_linear_pose_controller_parameters.set_signed_target_speed(true);
    passthrough_angular_pose_controller_parameters.set_signed_target_speed(true);
}

/// Disable linear pose control to avoid stopping once point is reached.
static void pf_quadpid_meta_controller_linear_pose_controller_disabled(void)
{
    // Restore quad PID controller to its original state
    pf_quadpid_meta_controller_restore();

    // Disable pose PID correction by using a passthrough controller
    linear_pose_loop_meta_controller.replace_controller(0, &passthrough_linear_pose_controller);
}

/// Disable angular correction and linear speed filtering for linear speed PID
/// setup.
static void pf_quadpid_meta_controller_linear_speed_controller_test_setup(void)
{
    // Restore quad PID controller to its original state
    pf_quadpid_meta_controller_restore();

    // Disable speed filter by setting maximum speed and acceleration to
    // unreachable limits
    linear_speed_filter_parameters.set_max_speed(etl::numeric_limits<uint16_t>::max());
    linear_speed_filter_parameters.set_max_acceleration(etl::numeric_limits<uint16_t>::max());

    // Disable pose PID correction by using a passthrough controller
    linear_pose_loop_meta_controller.replace_controller(0, &passthrough_linear_pose_controller);

    // Disable angular speed loop by using a PID with all gain set to zero
    angular_speed_controller_parameters.set_pid(&null_pid);

    // Do not sign target speed
    passthrough_linear_pose_controller_parameters.set_signed_target_speed(false);
}

/// Disable linear correction and angular speed filtering for angular speed PID
/// setup.
static void pf_quadpid_meta_controller_angular_speed_controller_test_setup(void)
{
    // Restore quad PID controller to its original state
    pf_quadpid_meta_controller_restore();

    // Disable speed filter by setting maximum speed and acceleration to
    // unreachable limits
    angular_speed_filter_parameters.set_max_speed(etl::numeric_limits<uint16_t>::max());
    angular_speed_filter_parameters.set_max_acceleration(etl::numeric_limits<uint16_t>::max());

    // Disable pose PID correction by using a passthrough controller
    angular_pose_loop_meta_controller.replace_controller(0, &passthrough_angular_pose_controller);

    // Disable linear speed loop by using a PID with all gain set to zero
    linear_speed_controller_parameters.set_pid(&null_pid);

    // Do not sign target speed
    passthrough_angular_pose_controller_parameters.set_signed_target_speed(false);
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
    LOG_INFO("Change to controller %" PRIu32 "\n", static_cast<uint32_t>(pb_controller.id()));
    current_controller_id = static_cast<uint32_t>(pb_controller.id());
    switch (static_cast<uint32_t>(pb_controller.id())) {
    case static_cast<uint32_t>(PB_ControllerEnum::LINEAR_SPEED_TEST):
        pf_quadpid_meta_controller_linear_speed_controller_test_setup();
        pf_motion_control_platform_engine.set_timeout_enable(true);
        break;

    case static_cast<uint32_t>(PB_ControllerEnum::ANGULAR_SPEED_TEST):
        pf_quadpid_meta_controller_angular_speed_controller_test_setup();
        pf_motion_control_platform_engine.set_timeout_enable(true);
        break;

    case static_cast<uint32_t>(PB_ControllerEnum::LINEAR_POSE_DISABLED):
        pf_quadpid_meta_controller_linear_pose_controller_disabled();
        pf_motion_control_platform_engine.set_timeout_enable(false);
        break;

    case static_cast<uint32_t>(PB_ControllerEnum::QUADPID):
    default:
        pf_quadpid_meta_controller_restore();
        pf_motion_control_platform_engine.set_timeout_enable(false);
        break;
    }

    pf_motion_control_platform_engine.set_current_cycle(0);
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
            linear_speed_filter.reset_previous_speed_order();
            angular_speed_filter.reset_previous_speed_order();
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
            // state
            pose_straight_filter.force_finished_state();

            // Reset previous speed orders
            linear_speed_filter.reset_previous_speed_order();
            angular_speed_filter.reset_previous_speed_order();

            LOG_WARNING("BLOCKED bypassed\n");

            pf_get_canpb().send_message(pose_reached_uuid);
        } else {
            // Stop motors
            left_motor.set_speed(0);
            right_motor.set_speed(0);

            // As motors are stopped, pose straight filter state machine is in
            // finished state
            pose_straight_filter.force_finished_state();

            // Reset previous speed orders
            linear_speed_filter.reset_previous_speed_order();
            angular_speed_filter.reset_previous_speed_order();

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
        qdecs_value[MOTOR_RIGHT] =
            (linear_speed_filter.previous_speed_order() * pulse_per_mm +
             angular_speed_filter.previous_speed_order() * pulse_per_degree / 2) *
            qdec_right_polarity.get();
        qdecs_value[MOTOR_LEFT] =
            (linear_speed_filter.previous_speed_order() * pulse_per_mm -
             angular_speed_filter.previous_speed_order() * pulse_per_degree / 2) *
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

void pf_handle_brake([[maybe_unused]] cogip::canpb::ReadBuffer& buffer)
{
    pf_motion_control_platform_engine.set_target_speed(cogip::cogip_defs::Polar(0, 0));
    reset_speed_pids();
    pose_straight_filter.force_finished_state();
}

void pf_handle_game_end([[maybe_unused]] cogip::canpb::ReadBuffer& buffer)
{
    pf_motion_control_platform_engine.set_target_speed(cogip::cogip_defs::Polar(0, 0));

    // Reset previous speed orders
    angular_speed_filter.reset_previous_speed_order();
    linear_speed_filter.reset_previous_speed_order();

    // Reset anti-blocking
    angular_speed_filter.reset_anti_blocking_blocked_cycles_nb();
    linear_speed_filter.reset_anti_blocking_blocked_cycles_nb();

    // Reset PIDs
    reset_speed_pids();

    // Force position filter finished state
    pose_straight_filter.force_finished_state();

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
        ? pose_straight_filter_parameters.bypass_final_orientation_on()
        : pose_straight_filter_parameters.bypass_final_orientation_off();

    // Set target speed for passthrough controllers
    passthrough_linear_pose_controller_parameters.set_target_speed(target_speed.distance());
    passthrough_angular_pose_controller_parameters.set_target_speed(target_speed.angle());

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
    angular_speed_filter.reset_previous_speed_order();
    linear_speed_filter.reset_previous_speed_order();
    // Reset anti-blocking
    angular_speed_filter.reset_anti_blocking_blocked_cycles_nb();
    linear_speed_filter.reset_anti_blocking_blocked_cycles_nb();

    // Reset PIDs
    reset_speed_pids();

    // Reset pose straight filter state
    pose_straight_filter.reset_current_state();
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

    // Associate a controller to the engine
    pf_motion_control_platform_engine.set_controller(pf_quadpid_meta_controller);

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
