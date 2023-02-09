// RIOT includes
#include "periph/qdec.h"

// Project includes
#include "app.hpp"
#include "app_conf.hpp"
#include "board.h"
#include "motion_control.hpp"
#include "path/Pose.hpp"
#include "platform.hpp"
#include "dualpid_meta_controller/DualPIDMetaController.hpp"
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

#include "PB_PathPose.hpp"
#include "PB_Pid.hpp"
#include "PB_State.hpp"

namespace cogip {

namespace pf {

namespace motion_control {

// Protobuf
PB_Pose pb_pose;
PB_State pb_state;
PB_Pids<cogip::pf::motion_control::PID_COUNT> pb_pids;

// Motion controllers
static cogip::motion_control::QuadPIDMetaController* pf_quadpid_meta_controller;
// Motion control engine
static cogip::motion_control::PlatformEngine pf_motion_control_platform_engine(
        cogip::motion_control::platform_get_speed_and_pose_cb_t::create<cogip::pf::motion_control::compute_current_speed_and_pose>(),
        cogip::motion_control::platform_process_commands_cb_t::create<cogip::pf::motion_control::pf_motor_drive>()
);

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


/// Initialize platform QuadPID meta controller
/// Return initialized QuadPID meta controller
static cogip::motion_control::QuadPIDMetaController* pf_quadpid_meta_controller_init(void) {
    // Filter controller behavior to always moves in a straight line
    static cogip::motion_control::PoseStraightFilterParameters pose_straight_filter_parameters = cogip::motion_control::PoseStraightFilterParameters(2, 2);
    static cogip::motion_control::PoseStraightFilter pose_straight_filter = cogip::motion_control::PoseStraightFilter(&pose_straight_filter_parameters);
    std::cout << "PoseStraightFilter created" << std::endl;

    // Split angular and linear controls
    static cogip::motion_control::PolarParallelMetaController polar_parallel_meta_controller;
    std::cout << "PolarParallelMetaController created" << std::endl;

    // Linear dual PID meta controller
    static cogip::motion_control::DualPIDMetaController linear_dualpid_meta_controller;
    polar_parallel_meta_controller.add_controller(&linear_dualpid_meta_controller);
    std::cout << "LinearDualPIDMetaController created and added to PolarParallelMetaController" << std::endl;
    static cogip::motion_control::PosePIDControllerParameters linear_position_controller_parameters(&linear_pose_pid);
    static cogip::motion_control::PosePIDController linear_position_controller(&linear_position_controller_parameters);
    linear_dualpid_meta_controller.add_controller(&linear_position_controller);
    std::cout << "PosePIDController created and added to LinearDualPIDMetaController" << std::endl;
    // Linear speed filter
    static cogip::motion_control::SpeedFilterParameters linear_speed_filter_parameters(
        cogip::pf::motion_control::platform_max_speed_linear_mm_per_period,
        cogip::pf::motion_control::platform_max_acc_linear_mm_per_period2
        );
    static cogip::motion_control::SpeedFilter linear_speed_filter(&linear_speed_filter_parameters);
    linear_dualpid_meta_controller.add_controller(&linear_speed_filter);
    std::cout << "SpeedFilter created and added to LinearDualPIDMetaController" << std::endl;
    static cogip::motion_control::SpeedPIDControllerParameters linear_speed_controller_parameters(&linear_speed_pid);
    static cogip::motion_control::SpeedPIDController linear_speed_controller(&linear_speed_controller_parameters);
    linear_dualpid_meta_controller.add_controller(&linear_speed_controller);
    std::cout << "SpeedPIDController created and added to LinearDualPIDMetaController" << std::endl;

    // Angular dual PID meta controller
    static cogip::motion_control::DualPIDMetaController angular_dualpid_meta_controller;
    polar_parallel_meta_controller.add_controller(&angular_dualpid_meta_controller);
    std::cout << "AngularDualPIDMetaController created and added to PolarParallelMetaController" << std::endl;
    static cogip::motion_control::PosePIDControllerParameters angular_position_controller_parameters(&angular_pose_pid);
    static cogip::motion_control::PosePIDController angular_position_controller(&angular_position_controller_parameters);
    angular_dualpid_meta_controller.add_controller(&angular_position_controller);
    std::cout << "PosePIDController created and added to AngularDualPIDMetaController" << std::endl;
    // Angular speed filter
    static cogip::motion_control::SpeedFilterParameters angular_speed_filter_parameters(
        cogip::pf::motion_control::platform_max_speed_angular_deg_per_period,
        cogip::pf::motion_control::platform_max_acc_angular_deg_per_period2
        );
    static cogip::motion_control::SpeedFilter angular_speed_filter(&angular_speed_filter_parameters);
    angular_dualpid_meta_controller.add_controller(&angular_speed_filter);
    std::cout << "SpeedFilter created and added to AngularDualPIDMetaController" << std::endl;
    static cogip::motion_control::SpeedPIDControllerParameters angular_speed_controller_parameters(&angular_speed_pid);
    static cogip::motion_control::SpeedPIDController angular_speed_controller(&angular_speed_controller_parameters);
    angular_dualpid_meta_controller.add_controller(&angular_speed_controller);
    std::cout << "SpeedPIDController created and added to AngularDualPIDMetaController" << std::endl;

    // Quad PID meta controller
    static cogip::motion_control::QuadPIDMetaController quadpid_meta_controller;
    quadpid_meta_controller.add_controller(&pose_straight_filter);
    quadpid_meta_controller.add_controller(&polar_parallel_meta_controller);

    return &quadpid_meta_controller;
}

void pf_print_state(void)
{
    COGIP_DEBUG_COUT(
        "{"
            << "\"pose_current\":{"
                << "\"O\":" << pf_motion_control_platform_engine.current_pose().O()
                << ",\"x\":" << pf_motion_control_platform_engine.current_pose().x()
                << ",\"y\":" << pf_motion_control_platform_engine.current_pose().y()
            << "},"
            << "\"cycle\":" << pf_motion_control_platform_engine.current_cycle() << ","
            << "\"speed_current\":{\"distance\":" << pf_motion_control_platform_engine.current_speed().distance()
            << ",\"angle\":" << pf_motion_control_platform_engine.current_speed().angle() << "}"
    );

    COGIP_DEBUG_COUT("}");
}

void pf_send_pb_pose(void)
{
    pf_motion_control_platform_engine.current_pose().pb_copy(pb_pose);
    pf_get_uartpb().send_message(cogip::pf::motion_control::pose_uuid, &pb_pose);
}

void pf_send_pids(void)
{
    // Linear pose PID
    linear_pose_pid.pb_copy(pb_pids.mutable_pids((uint32_t)PB_PidEnum::LINEAR_POSE_PID));
    pb_pids.mutable_pids((uint32_t)PB_PidEnum::LINEAR_POSE_PID).set_id(PB_PidEnum::LINEAR_POSE_PID);

    // Angular pose PID
    angular_pose_pid.pb_copy(pb_pids.mutable_pids((uint32_t)PB_PidEnum::ANGULAR_POSE_PID));
    pb_pids.mutable_pids((uint32_t)PB_PidEnum::ANGULAR_POSE_PID).set_id(PB_PidEnum::ANGULAR_POSE_PID);

    // Linear speed PID
    linear_speed_pid.pb_copy(pb_pids.mutable_pids((uint32_t)PB_PidEnum::LINEAR_SPEED_PID));
    pb_pids.mutable_pids((uint32_t)PB_PidEnum::LINEAR_SPEED_PID).set_id(PB_PidEnum::LINEAR_SPEED_PID);

    // Angular speed PID
    angular_speed_pid.pb_copy(pb_pids.mutable_pids((uint32_t)PB_PidEnum::ANGULAR_SPEED_PID));
    pb_pids.mutable_pids((uint32_t)PB_PidEnum::ANGULAR_SPEED_PID).set_id(PB_PidEnum::ANGULAR_SPEED_PID);

    pb_state.clear();
    pb_state.mutable_cycle() = pf_motion_control_platform_engine.current_cycle();
    pf_motion_control_platform_engine.current_speed().pb_copy(pb_state.mutable_speed_current());
    pf_motion_control_platform_engine.target_speed().pb_copy(pb_state.mutable_speed_order());

    pf_get_uartpb().send_message(cogip::pf::motion_control::pid_uuid, &pb_pids);
}

void pf_send_pb_state(void)
{
    pb_state.clear();
    pb_state.mutable_cycle() = pf_motion_control_platform_engine.current_cycle();
    pf_motion_control_platform_engine.current_speed().pb_copy(pb_state.mutable_speed_current());
    pf_motion_control_platform_engine.target_speed().pb_copy(pb_state.mutable_speed_order());

    pf_get_uartpb().send_message(cogip::pf::motion_control::state_uuid, &pb_state);
}

void pf_handle_target_pose(cogip::uartpb::ReadBuffer &buffer)
{
    PB_PathPose pb_pose_to_reach;
    EmbeddedProto::Error error = pb_pose_to_reach.deserialize(buffer);
    if (error != EmbeddedProto::Error::NO_ERRORS) {
        std::cout << "Pose to reach: Protobuf deserialization error: " << static_cast<int>(error) << std::endl;
        return;
    }
    cogip::path::Pose pose_to_reach;
    pose_to_reach.pb_read(pb_pose_to_reach);
    cogip::cogip_defs::Pose pose(pose_to_reach.x(), pose_to_reach.y(), pose_to_reach.O());
    cogip::cogip_defs::Polar speed_order;
    speed_order.set_distance(pose_to_reach.max_speed_linear());
    speed_order.set_angle(pose_to_reach.max_speed_angular());

    pf_motion_control_platform_engine.set_target_pose(pose);
    pf_motion_control_platform_engine.set_target_speed(speed_order);
    pf_motion_control_platform_engine.set_allow_reverse(pose_to_reach.allow_reverse());
    pf_motion_control_platform_engine.set_pose_reached(cogip::motion_control::target_pose_status_t::moving);
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
    cogip::cogip_defs::Pose pose(start_pose.x(), start_pose.y(), start_pose.O());

    pf_motion_control_platform_engine.set_current_pose(pose);
    pf_motion_control_platform_engine.set_target_pose(pose);
}

void pf_start_motion_control(void)
{
    // Start engine thread
    pf_motion_control_platform_engine.start_thread();
}

void pf_encoder_read(cogip::cogip_defs::Polar &current_speed)
{
    int32_t left_speed = qdec_read_and_reset(MOTOR_LEFT) * QDEC_LEFT_POLARITY;
    int32_t right_speed = qdec_read_and_reset(MOTOR_RIGHT) * QDEC_RIGHT_POLARITY;

    /* update speed */
    current_speed.set_distance(((right_speed + left_speed) / 2.0) / cogip::pf::motion_control::pulse_per_mm);
    current_speed.set_angle((right_speed - left_speed) / cogip::pf::motion_control::pulse_per_degree);
}

static void pf_encoder_reset(void)
{
    qdec_read_and_reset(MOTOR_LEFT);
    qdec_read_and_reset(MOTOR_RIGHT);
}

void compute_current_speed_and_pose(cogip::cogip_defs::Polar &current_speed, cogip::cogip_defs::Pose &current_pose)
{
    pf_encoder_read(current_speed);
    odometry_update(current_pose, current_speed, SEGMENT);
}


void pf_motor_drive(const cogip::cogip_defs::Polar &command)
{
    int16_t right_command = 0;
    int16_t left_command = 0;

    if (pf_motion_control_platform_engine.pose_reached() == cogip::motion_control::target_pose_status_t::moving) {
        // Compute motor commands with Polar motion control result
        right_command = (int16_t) (command.distance() + command.angle());
        left_command = (int16_t) (command.distance() - command.angle());
    }
    else {
        // Reset speed PIDs if a pose has been reached (intermediate or final)
        // Pose PIDs do not need to be reset as they only have Kp (no sum of error)
        linear_speed_pid.reset();
        angular_speed_pid.reset();

        // Send message in case of final pose reached only
        if (pf_motion_control_platform_engine.pose_reached() == cogip::motion_control::target_pose_status_t::reached) {
            pf_get_uartpb().send_message(cogip::pf::motion_control::pose_reached_uuid);
        }
    }

    // Apply motor commands
    motor_set(MOTOR_DRIVER_DEV(0), MOTOR_LEFT, left_command);
    motor_set(MOTOR_DRIVER_DEV(0), MOTOR_RIGHT, right_command);

    // Send robot state
    pf_send_pb_state();
}

/// Handle pid request command message.
static void _handle_pid_request([[maybe_unused]] cogip::uartpb::ReadBuffer & buffer)
{
    // Send PIDs
    pf_send_pids();
}

void pf_init_motion_control(void)
{
    // Init motor driver
    motor_driver_init(MOTOR_DRIVER_DEV(0));

    // Setup qdec periphereal
    int error = qdec_init(QDEC_DEV(MOTOR_LEFT), QDEC_MODE, NULL, NULL);
    if (error) {
        printf("QDEC %u not initialized, error=%d !!!\n", MOTOR_LEFT, error);
    }
    error = qdec_init(QDEC_DEV(MOTOR_RIGHT), QDEC_MODE, NULL, NULL);
    if (error) {
        printf("QDEC %u not initialized, error=%d !!!\n", MOTOR_RIGHT, error);
    }

    // Init odometry
    odometry_setup(cogip::pf::motion_control::wheels_distance / cogip::pf::motion_control::pulse_per_mm);

    //TODO: update
    //ctrl_set_anti_blocking_on(pf_get_ctrl(), TRUE);

    // Init controllers
    pf_quadpid_meta_controller = pf_quadpid_meta_controller_init();

    // Associate a controller to the engine
    pf_motion_control_platform_engine.set_controller(pf_quadpid_meta_controller);

    // Register pid request command
    pf_get_uartpb().register_message_handler(
        cogip::pf::motion_control::pid_request_uuid,
        cogip::uartpb::message_handler_t::create<_handle_pid_request>()
    );
}

} // namespace actuators

} // namespace pf

} // namespace cogip