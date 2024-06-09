// RIOT includes
#include "periph/qdec.h"

// Project includes
#include "app.hpp"
#include "app_conf.hpp"
#include "board.h"
#include "motion_control.hpp"
#include "motion_motors_params.hpp"
#include "path/Pose.hpp"
#include "platform.hpp"
#include "dualpid_meta_controller/DualPIDMetaController.hpp"
#include "passthrough_pose_pid_controller/PassthroughPosePIDController.hpp"
#include "passthrough_pose_pid_controller/PassthroughPosePIDControllerParameters.hpp"
#include "platform_engine/PlatformEngine.hpp"
#include "polar_parallel_meta_controller/PolarParallelMetaController.hpp"
#include "pose_pid_controller/PosePIDController.hpp"
#include "pose_pid_controller/PosePIDControllerParameters.hpp"
#include "pose_straight_filter/PoseStraightFilter.hpp"
#include "pose_straight_filter/PoseStraightFilterParameters.hpp"
#include "quadpid_meta_controller/QuadPIDMetaController.hpp"
#include "speed_filter/SpeedFilter.hpp"
#include "speed_filter/SpeedFilterParameters.hpp"
#include "speed_pid_controller/SpeedPIDController.hpp"
#include "speed_pid_controller/SpeedPIDControllerParameters.hpp"

#include "PB_Controller.hpp"
#include "PB_PathPose.hpp"
#include "PB_Pid.hpp"
#include "PB_State.hpp"

namespace cogip {

namespace pf {

namespace motion_control {

// Motion control motor driver
static motor_driver_t motion_motors_driver;

// Current controller
static uint32_t current_controller_id = 0;

// Protobuf
PB_Pose pb_pose;
PB_Controller pb_controller;
PB_State pb_state;
PB_Pid pb_pid;
PB_Pid_Id pb_pid_id;


// PID tuning period
constexpr uint16_t motion_control_pid_tuning_period_ms = 3000;

// Motion controllers
static cogip::motion_control::QuadPIDMetaController* pf_quadpid_meta_controller;
// Motion control engine
static cogip::motion_control::PlatformEngine pf_motion_control_platform_engine(
        cogip::motion_control::platform_get_speed_and_pose_cb_t::create<compute_current_speed_and_pose>(),
        cogip::motion_control::platform_process_commands_cb_t::create<pf_motor_drive>()
);
// Target pose
static cogip::path::Pose target_pose;
// Target speed
static cogip::cogip_defs::Polar target_speed;

// Linear pose PID controller
static cogip::pid::PID linear_pose_pid(
    linear_pose_pid_kp,
    linear_pose_pid_ki,
    linear_pose_pid_kd,
    linear_pose_pid_integral_limit
    );
// Linear speed PID controller
static cogip::pid::PID linear_speed_pid(
    linear_speed_pid_kp,
    linear_speed_pid_ki,
    linear_speed_pid_kd,
    linear_speed_pid_integral_limit
    );
// Angular pose PID controller
static cogip::pid::PID angular_pose_pid(
    angular_pose_pid_kp,
    angular_pose_pid_ki,
    angular_pose_pid_kd,
    angular_pose_pid_integral_limit
);
// Angular speed PID controller
static cogip::pid::PID angular_speed_pid(
    angular_speed_pid_kp,
    angular_speed_pid_ki,
    angular_speed_pid_kd,
    angular_speed_pid_integral_limit
    );
// PID null
static cogip::pid::PID null_pid(0, 0, 0, 0);

static void reset_speed_pids() {
    angular_speed_pid.reset();
    linear_speed_pid.reset();
}

/// PoseStraightFilter parameters.
static cogip::motion_control::PoseStraightFilterParameters pose_straight_filter_parameters =
        cogip::motion_control::PoseStraightFilterParameters(
            angular_threshold,
            linear_threshold,
            angular_intermediate_threshold,
            platform_max_dec_angular_deg_per_period2,
            platform_max_dec_linear_mm_per_period2);
/// PoseStraightFilter controller to make the robot always moves in a straight line.
static cogip::motion_control::PoseStraightFilter pose_straight_filter =
        cogip::motion_control::PoseStraightFilter(&pose_straight_filter_parameters);
/// PolarParallelMetaController to split linear and angular chain.
static cogip::motion_control::PolarParallelMetaController polar_parallel_meta_controller;

/// Linear DualPIDMetaController for pose and speed control in cascade.
static cogip::motion_control::DualPIDMetaController linear_dualpid_meta_controller;
/// Linear PosePIDControllerParameters.
static cogip::motion_control::PosePIDControllerParameters linear_pose_controller_parameters(&linear_pose_pid);
/// Linear PosePIDController that provides SpeedPIDController order first filtered by SpeedFilter.
static cogip::motion_control::PosePIDController linear_pose_controller(&linear_pose_controller_parameters);
/// Linear SpeedFilterParameters.
static cogip::motion_control::SpeedFilterParameters linear_speed_filter_parameters(
    platform_min_speed_linear_mm_per_period,
    platform_max_speed_linear_mm_per_period,
    platform_max_acc_linear_mm_per_period2,
    false
    );
/// Linear SpeedFilter to limit speed and acceleration for linear SpeedPIDController.
static cogip::motion_control::SpeedFilter linear_speed_filter(&linear_speed_filter_parameters);
/// Linear SpeedPIDControllerParameters.
static cogip::motion_control::SpeedPIDControllerParameters linear_speed_controller_parameters(&linear_speed_pid);
/// Linear SpeedPIDController to compute linear command to send to motors.
static cogip::motion_control::SpeedPIDController linear_speed_controller(&linear_speed_controller_parameters);

/// Angular DualPIDMetaController for pose and speed control in cascade.
static cogip::motion_control::DualPIDMetaController angular_dualpid_meta_controller;
/// Angular PosePIDControllerParameters.
static cogip::motion_control::PosePIDControllerParameters angular_pose_controller_parameters(&angular_pose_pid);
/// Angular PosePIDController that provides SpeedPIDController order first filtered by SpeedFilter.
static cogip::motion_control::PosePIDController angular_pose_controller(&angular_pose_controller_parameters);
/// Angular SpeedFilterParameters.
static cogip::motion_control::SpeedFilterParameters angular_speed_filter_parameters(
    platform_min_speed_angular_deg_per_period,
    platform_max_speed_angular_deg_per_period,
    platform_max_acc_angular_deg_per_period2
    );
/// Angular SpeedFilter to limit speed and acceleration for angular SpeedPIDController.
static cogip::motion_control::SpeedFilter angular_speed_filter(&angular_speed_filter_parameters);
/// Angular SpeedPIDControllerParameters.
static cogip::motion_control::SpeedPIDControllerParameters angular_speed_controller_parameters(&angular_speed_pid);
/// Angular SpeedPIDController to compute angular command to send to motors.
static cogip::motion_control::SpeedPIDController angular_speed_controller(&angular_speed_controller_parameters);

/// Linear PassthroughPosePIDControllerParameters.
static cogip::motion_control::PassthroughPosePIDControllerParameters passthrough_linear_pose_controller_parameters(
    platform_max_speed_linear_mm_per_period,
    true
    );
/// Linear PassthroughPosePIDController replaces linear PosePIDController to bypass it, imposing target speed as speed order.
static cogip::motion_control::PassthroughPosePIDController passthrough_linear_pose_controller(&passthrough_linear_pose_controller_parameters);
/// Angular PassthroughPosePIDControllerParameters.
static cogip::motion_control::PassthroughPosePIDControllerParameters passthrough_angular_pose_controller_parameters(
    platform_max_speed_angular_deg_per_period,
    true
    );
/// Angular PassthroughPosePIDController replaces angular PosePIDController to bypass it, imposing target speed as speed order.
static cogip::motion_control::PassthroughPosePIDController passthrough_angular_pose_controller(&passthrough_angular_pose_controller_parameters);

/// Quad PID meta controller.
static cogip::motion_control::QuadPIDMetaController quadpid_meta_controller;

/// Initialize platform QuadPID meta controller
/// Return initialized QuadPID meta controller
static cogip::motion_control::QuadPIDMetaController* pf_quadpid_meta_controller_init(void) {

    // Linear DualPIDMetaController
    //  PosePIDController -> SpeedFilter -> SpeedPIDController
    linear_dualpid_meta_controller.add_controller(&linear_pose_controller);
    linear_dualpid_meta_controller.add_controller(&linear_speed_filter);
    linear_dualpid_meta_controller.add_controller(&linear_speed_controller);

    // Angular DualPIDMetaController:
    //  PosePIDController -> SpeedFilter -> SpeedPIDController
    angular_dualpid_meta_controller.add_controller(&angular_pose_controller);
    angular_dualpid_meta_controller.add_controller(&angular_speed_filter);
    angular_dualpid_meta_controller.add_controller(&angular_speed_controller);

    // ParallelMetaController:
    // --> Linear DualPIDMetaController
    // `-> Angular DualPIDMetaController
    polar_parallel_meta_controller.add_controller(&linear_dualpid_meta_controller);
    polar_parallel_meta_controller.add_controller(&angular_dualpid_meta_controller);

    // QuadPIDMetaController:
    // PoseStraightFilter -> PolarParallelMetaController
    quadpid_meta_controller.add_controller(&pose_straight_filter);
    quadpid_meta_controller.add_controller(&polar_parallel_meta_controller);

    return &quadpid_meta_controller;
}

/// Restore platform QuadPID meta controller to its original configuration.
static void pf_quadpid_meta_controller_restore(void) {
    // Linear speed limits
    linear_speed_filter_parameters.set_max_speed(platform_max_speed_linear_mm_per_period);
    linear_speed_filter_parameters.set_max_acceleration(platform_max_acc_linear_mm_per_period2);
    // Angular speed limits
    angular_speed_filter_parameters.set_max_speed(platform_max_speed_angular_deg_per_period);
    angular_speed_filter_parameters.set_max_acceleration(platform_max_acc_angular_deg_per_period2);

    // Linear dual PID meta controller
    linear_dualpid_meta_controller.replace_controller(0, &linear_pose_controller);
    // Angular dual PID meta controller
    angular_dualpid_meta_controller.replace_controller(0, &angular_pose_controller);

    // Linear speed PID controller parameters
    linear_speed_controller_parameters.set_pid(&linear_speed_pid);
    // Angular speed PID controller parameters
    angular_speed_controller_parameters.set_pid(&angular_speed_pid);

    // Sign target speed according to pose error
    passthrough_linear_pose_controller_parameters.set_signed_target_speed(true);
    passthrough_angular_pose_controller_parameters.set_signed_target_speed(true);
}

/// Disable linear pose control to avoid stopping once point is reached.
static void pf_quadpid_meta_controller_linear_pose_controller_disabled(void) {
    // Restore quad PID controller to its original state
    pf_quadpid_meta_controller_restore();

    // Disable pose PID correction by using a passthrough controller
    linear_dualpid_meta_controller.replace_controller(0, &passthrough_linear_pose_controller);
}

/// Disable angular correction and linear speed filtering for linear speed PID setup.
static void pf_quadpid_meta_controller_linear_speed_controller_test_setup(void) {
    // Restore quad PID controller to its original state
    pf_quadpid_meta_controller_restore();

    // Disable speed filter by setting maximum speed and acceleration to unreachable limits
    linear_speed_filter_parameters.set_max_speed(std::numeric_limits<uint16_t>::max());
    linear_speed_filter_parameters.set_max_acceleration(std::numeric_limits<uint16_t>::max());

    // Disable pose PID correction by using a passthrough controller
    linear_dualpid_meta_controller.replace_controller(0, &passthrough_linear_pose_controller);

    // Disable angular speed loop by using a PID with all gain set to zero
    angular_speed_controller_parameters.set_pid(&null_pid);

    // Do not sign target speed
    passthrough_linear_pose_controller_parameters.set_signed_target_speed(false);
}

/// Disable linear correction and angular speed filtering for angular speed PID setup.
static void pf_quadpid_meta_controller_angular_speed_controller_test_setup(void) {
    // Restore quad PID controller to its original state
    pf_quadpid_meta_controller_restore();

    // Disable speed filter by setting maximum speed and acceleration to unreachable limits
    angular_speed_filter_parameters.set_max_speed(std::numeric_limits<uint16_t>::max());
    angular_speed_filter_parameters.set_max_acceleration(std::numeric_limits<uint16_t>::max());

    // Disable pose PID correction by using a passthrough controller
    angular_dualpid_meta_controller.replace_controller(0, &passthrough_angular_pose_controller);

    // Disable linear speed loop by using a PID with all gain set to zero
    linear_speed_controller_parameters.set_pid(&null_pid);

    // Do not sign target speed
    passthrough_angular_pose_controller_parameters.set_signed_target_speed(false);
}

/// Update current speed from quadrature encoders measure.
void pf_encoder_read(cogip::cogip_defs::Polar &current_speed)
{
    int32_t left_speed = qdec_read_and_reset(MOTOR_LEFT) * QDEC_LEFT_POLARITY;
    int32_t right_speed = qdec_read_and_reset(MOTOR_RIGHT) * QDEC_RIGHT_POLARITY;

    // update speed
    current_speed.set_distance(((right_speed + left_speed) / 2.0) / pulse_per_mm);
    current_speed.set_angle((right_speed - left_speed) / pulse_per_degree);
}

static void pf_encoder_reset(void)
{
    qdec_read_and_reset(MOTOR_LEFT);
    qdec_read_and_reset(MOTOR_RIGHT);
}

void pf_send_pb_pose(void)
{
    pf_motion_control_platform_engine.current_pose().pb_copy(pb_pose);
    pf_get_uartpb().send_message(pose_uuid, &pb_pose);
}

void pf_send_pid(PB_PidEnum id)
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

    pf_get_uartpb().send_message(pid_uuid, &pb_pid);
}

void pf_send_pb_state(void)
{
    pb_state.clear();
    pb_state.mutable_cycle() = pf_motion_control_platform_engine.current_cycle();
    pf_motion_control_platform_engine.current_speed().pb_copy(pb_state.mutable_speed_current());
    pf_motion_control_platform_engine.target_speed().pb_copy(pb_state.mutable_speed_order());

    pf_get_uartpb().send_message(state_uuid, &pb_state);
}


void pf_handle_brake([[maybe_unused]] cogip::uartpb::ReadBuffer &buffer) {
    pf_motion_control_platform_engine.set_target_speed({0, 0});
    reset_speed_pids();
    pose_straight_filter.force_finished_state();
}

void pf_handle_target_pose(cogip::uartpb::ReadBuffer &buffer)
{
    // Retrieve new target pose from protobuf message
    PB_PathPose pb_path_target_pose;
    EmbeddedProto::Error error = pb_path_target_pose.deserialize(buffer);
    if (error != EmbeddedProto::Error::NO_ERRORS) {
        std::cout << "Pose to reach: Protobuf deserialization error: " << static_cast<int>(error) << std::endl;
        return;
    }

    cogip::path::Pose previous_target_pose = target_pose;

    // Target pose
    target_pose.pb_read(pb_path_target_pose);

    // Target speed
    target_speed.set_distance((platform_max_speed_linear_mm_per_period * target_pose.max_speed_ratio_linear()) / 100);
    target_speed.set_angle((platform_max_speed_angular_deg_per_period * target_pose.max_speed_ratio_angular()) / 100);
    pf_motion_control_platform_engine.set_target_speed(target_speed);

    // Set final orientation bypassing
    target_pose.bypass_final_orientation()
        ? pose_straight_filter_parameters.bypass_final_orientation_on()
        : pose_straight_filter_parameters.bypass_final_orientation_off();

    // Set target speed for passthrough controllers
    passthrough_linear_pose_controller_parameters.set_target_speed(target_speed.distance());
    passthrough_angular_pose_controller_parameters.set_target_speed(target_speed.angle());

    if (target_pose.timeout_ms()) {
        pf_motion_control_platform_engine.set_timeout_enable(true);
        pf_motion_control_platform_engine.set_timeout_cycle_number(target_pose.timeout_ms() / motion_control_thread_period_ms);
    }
    else {
        pf_motion_control_platform_engine.set_timeout_enable(false);
    }

    // Deal with the first pose in the list
    pf_motion_control_platform_engine.set_target_pose(target_pose);

    // New target pose, the robot is moving
    pf_motion_control_platform_engine.set_pose_reached(cogip::motion_control::target_pose_status_t::moving);

    pf_motion_control_reset();

    pf_enable_motion_control();
}

void pf_handle_start_pose(cogip::uartpb::ReadBuffer &buffer)
{
    PB_PathPose pb_start_pose;
    EmbeddedProto::Error error = pb_start_pose.deserialize(buffer);
    if (error != EmbeddedProto::Error::NO_ERRORS) {
        std::cout << "Current pose: Protobuf deserialization error: " << static_cast<int>(error) << std::endl;
        return;
    }
    cogip::path::Pose start_pose;
    start_pose.pb_read(pb_start_pose);
    cogip::path::Pose pose(start_pose.x(), start_pose.y(), start_pose.O());

    pf_motion_control_platform_engine.set_current_pose(pose);
    pf_motion_control_platform_engine.set_target_pose(pose);
    // New start pose, the robot is not moving
    pf_motion_control_platform_engine.set_pose_reached(cogip::motion_control::target_pose_status_t::reached);

    pf_encoder_reset();
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

    pf_encoder_reset();
}

void pf_disable_motion_control()
{
    pf_motion_control_platform_engine.disable();

    // Small wait to ensure engine is disabled
    ztimer_sleep(ZTIMER_MSEC, motion_control_thread_period_ms);

    // Stop motors as the robot should not move in this case.
    motor_disable(&motion_motors_driver, MOTOR_LEFT);
    motor_disable(&motion_motors_driver, MOTOR_RIGHT);
}

void pf_enable_motion_control()
{
    pf_motion_control_platform_engine.enable();
}

void compute_current_speed_and_pose(cogip::cogip_defs::Polar &current_speed, cogip::cogip_defs::Pose &current_pose)
{
    pf_encoder_read(current_speed);
    odometry_update(current_pose, current_speed, SEGMENT);
}

void pf_motor_drive(const cogip::cogip_defs::Polar &command)
{
    // Previous target pose status flag to avoid flooding protobuf serial bus.
    static cogip::motion_control::target_pose_status_t previous_target_pose_status = cogip::motion_control::target_pose_status_t::moving;

    // Limit commands to what the PWM driver can accept as input in the range [INT16_MIN:INT16_MAX].
    // The PWM driver will filter the value to the max PWM resolution defined for the board.
    // Compute motor commands with Polar motion control result
    int16_t right_command = (int16_t) std::max(std::min(command.distance() + command.angle(), (double)(std::numeric_limits<int16_t>::max()) / 2),
                                               (double)(std::numeric_limits<int16_t>::min()) / 2);
    int16_t left_command = (int16_t) std::max(std::min(command.distance() - command.angle(), (double)(std::numeric_limits<int16_t>::max()) / 2),
                                                  (double)(std::numeric_limits<int16_t>::min()) / 2);

    if (pf_motion_control_platform_engine.pose_reached() == cogip::motion_control::target_pose_status_t::blocked) {
        if (pf_motion_control_platform_engine.target_pose().bypass_anti_blocking()) {
            target_pose.set_x(pf_motion_control_platform_engine.current_pose().x());
            target_pose.set_y(pf_motion_control_platform_engine.current_pose().y());
            target_pose.set_O(pf_motion_control_platform_engine.current_pose().O());
            pf_motion_control_platform_engine.set_target_pose(target_pose);

            // Consider pose_reached as anti blocking is bypassed
            pf_motion_control_platform_engine.set_pose_reached(cogip::motion_control::target_pose_status_t::reached);
            // As pose is reached, pose straight filter state machine is in finished state
            pose_straight_filter.force_finished_state();

            std::cout << "BLOCKED bypasssed" << std::endl;
        }
        else {
            right_command = 0;
            left_command = 0;

            pose_straight_filter.force_finished_state();

            std::cout << "BLOCKED" << std::endl;
        }
    }
    else {
        // Apply motor commands
        if (fabs(right_command) > motion_motors_driver.params->pwm_resolution) {
            right_command = (fabs(right_command)/right_command) * motion_motors_driver.params->pwm_resolution - 1;
        }
        if (fabs(left_command) > motion_motors_driver.params->pwm_resolution) {
            left_command = (fabs(left_command)/left_command) * motion_motors_driver.params->pwm_resolution - 1;
        }
        right_command = (right_command < 0 ? -pwm_minimal : pwm_minimal )
                        + ((right_command * (int16_t)(motion_motors_driver.params->pwm_resolution - pwm_minimal))
                            / (int16_t)motion_motors_driver.params->pwm_resolution);
        left_command = (left_command < 0 ? -pwm_minimal : pwm_minimal)
                        + ((left_command * (int16_t)(motion_motors_driver.params->pwm_resolution - pwm_minimal))
                            / (int16_t)motion_motors_driver.params->pwm_resolution);
    }

    motor_set(&motion_motors_driver, MOTOR_RIGHT, right_command);
    motor_set(&motion_motors_driver, MOTOR_LEFT, left_command);

    if (pf_motion_control_platform_engine.pose_reached() == cogip::motion_control::target_pose_status_t::reached) {
        // Send message in case of final pose reached only.
        if ((pf_motion_control_platform_engine.pose_reached() == cogip::motion_control::target_pose_status_t::reached)
            &&  (previous_target_pose_status != cogip::motion_control::target_pose_status_t::reached)) {
            pf_get_uartpb().send_message(pose_reached_uuid);
        }

        // Reset previous speed orders
        linear_speed_filter.reset_previous_speed_order();
        angular_speed_filter.reset_previous_speed_order();
    }

    if ((pf_motion_control_platform_engine.pose_reached() != cogip::motion_control::target_pose_status_t::moving)
        && (pf_motion_control_platform_engine.pose_reached() != previous_target_pose_status)) {
        // Reset speed PIDs if a pose has been reached (intermediate or final).
        // Pose PIDs do not need to be reset as they only have Kp (no sum of error).
        reset_speed_pids();
    }

    // Backup target pose status flag to avoid flooding protobuf serial bus.
    previous_target_pose_status = pf_motion_control_platform_engine.pose_reached();
}

/// Handle pid request command message.
static void _handle_pid_request([[maybe_unused]] cogip::uartpb::ReadBuffer & buffer)
{
    pb_pid_id.clear();
    EmbeddedProto::Error error = pb_pid_id.deserialize(buffer);
    if (error != EmbeddedProto::Error::NO_ERRORS) {
        std::cout << "Pid request: Protobuf deserialization error: " << static_cast<int>(error) << std::endl;
        return;
    }
    else {
        std::cout << "Pid request: " << static_cast<uint32_t>(pb_pid_id.id()) << std::endl;
    }

    // Send PIDs
    pf_send_pid(pb_pid_id.id());
}

/// Handle new pid config message.
static void _handle_new_pid_config(cogip::uartpb::ReadBuffer & buffer)
{
    pb_pid.clear();

    EmbeddedProto::Error error = pb_pid.deserialize(buffer);
    if (error != EmbeddedProto::Error::NO_ERRORS) {
        std::cout << "New pid config: Protobuf deserialization error: " << static_cast<int>(error) << std::endl;
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
static void _handle_set_controller(cogip::uartpb::ReadBuffer & buffer)
{
    pb_controller.clear();

    EmbeddedProto::Error error = pb_controller.deserialize(buffer);
    if (error != EmbeddedProto::Error::NO_ERRORS) {
        std::cout << "Controller change request: Protobuf deserialization error: " << static_cast<int>(error) << std::endl;
        return;
    }

    // Change controller
    std::cout << "Change to controller " << static_cast<uint32_t>(pb_controller.id()) << std::endl;
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

extern "C" {
extern int32_t qdecs_value[QDEC_NUMOF];
}

void cogip_native_motor_driver_qdec_simulation(
    const motor_driver_t *motor_driver, uint8_t motor_id,
    int32_t pwm_duty_cycle)
{
    // Unused variables
    (void) motor_driver;
    (void) motor_id;
    (void) pwm_duty_cycle;

    // On native architecture set speeds at their theorical value, no error.
    if (pf_motion_control_platform_engine.pose_reached() != cogip::motion_control::target_pose_status_t::reached) {
        qdecs_value[MOTOR_RIGHT] = (linear_speed_filter.previous_speed_order() * pulse_per_mm
                                   + angular_speed_filter.previous_speed_order() * pulse_per_degree / 2)
                                   * QDEC_RIGHT_POLARITY;
        qdecs_value[MOTOR_LEFT] = (linear_speed_filter.previous_speed_order() * pulse_per_mm
                                  - angular_speed_filter.previous_speed_order() * pulse_per_degree / 2)
                                  * QDEC_LEFT_POLARITY;
    }
}

void pf_init_motion_control(void)
{
    // Init motor driver
    motor_driver_init(&motion_motors_driver, &motion_motors_params);

    // Setup qdec periphereal
    int error = qdec_init(QDEC_DEV(MOTOR_LEFT), QDEC_MODE, NULL, NULL);
    if (error) {
        printf("QDEC %u not initialized, error=%d !!!\n", MOTOR_LEFT, error);
    }
    error = qdec_init(QDEC_DEV(MOTOR_RIGHT), QDEC_MODE, NULL, NULL);
    if (error) {
        printf("QDEC %u not initialized, error=%d !!!\n", MOTOR_RIGHT, error);
    }

    // Init controllers
    pf_quadpid_meta_controller = pf_quadpid_meta_controller_init();

    // Associate a controller to the engine
    pf_motion_control_platform_engine.set_controller(pf_quadpid_meta_controller);

    // Set timeout for speed only loops as no pose has to be reached
    pf_motion_control_platform_engine.set_timeout_cycle_number(
        motion_control_pid_tuning_period_ms/motion_control_thread_period_ms);

    // Register pid request command
    pf_get_uartpb().register_message_handler(
        pid_request_uuid,
        cogip::uartpb::message_handler_t::create<_handle_pid_request>()
    );

    // Register new pids config
    pf_get_uartpb().register_message_handler(
        pid_uuid,
        cogip::uartpb::message_handler_t::create<_handle_new_pid_config>()
    );

    // Register new pids config
    pf_get_uartpb().register_message_handler(
        controller_uuid,
        cogip::uartpb::message_handler_t::create<_handle_set_controller>()
    );

    pf_encoder_reset();
    pf_disable_motion_control();
}

} // namespace actuators

} // namespace pf

} // namespace cogip
