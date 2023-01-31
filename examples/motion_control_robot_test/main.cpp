// RIOT includes
#include <shell.h>
#include <ztimer.h>

// System includes
#include <cstdio>
#include <iostream>

#include "etl/list.h"
#include "etl/vector.h"

#include "pid_ng/PID.hpp"

#include "motion_control_common/Controller.hpp"
#include "dualpid_meta_controller/DualPIDMetaController.hpp"
#include "quadpid_meta_controller/QuadPIDMetaController.hpp"
#include "platform_engine/PlatformEngine.hpp"
#include "pose_pid_controller/PosePIDController.hpp"
#include "speed_pid_controller/SpeedPIDController.hpp"
#include "pose_straight_filter/PoseStraightFilter.hpp"
#include "speed_filter/SpeedFilter.hpp"
#include "polar_parallel_meta_controller/PolarParallelMetaController.hpp"


void robot_speeds(cogip::cogip_defs::Polar& speed_current,
    cogip::cogip_defs::Polar& speed_target)
{
    static double i = 0;

    speed_current.set_distance(i++);
    speed_current.set_angle(i++);
    speed_target.set_distance(i++);
    speed_target.set_angle(i++);
}

void robot_poses(cogip::cogip_defs::Pose& pose_current,
    cogip::cogip_defs::Pose& pose_target)
{
    static uint32_t i = 0;

    pose_current.set_x(i++);
    pose_current.set_y(i++);
    pose_current.set_O(i++);
    pose_target.set_x(i++);
    pose_target.set_y(i++);
    pose_target.set_O(i++);
}

cogip::motion_control::PlatformEngine engine(
    cogip::motion_control::platform_get_poses_cb_t::create<robot_poses>(),
    cogip::motion_control::platform_get_speeds_cb_t::create<robot_speeds>()
);

int main(void)
{
    puts("\n== Controller skeleton ==");

    // Filter controller behavior to always moves in a straight line
    cogip::motion_control::PoseStraightFilterParameters pose_straight_filter_parameters = cogip::motion_control::PoseStraightFilterParameters(2, 2);
    cogip::motion_control::PoseStraightFilter pose_straight_filter = cogip::motion_control::PoseStraightFilter(&pose_straight_filter_parameters);
    std::cout << "PoseStraightFilter created" << std::endl;

    // Split angular and linear controls
    cogip::motion_control::PolarParallelMetaController polar_parallel_meta_controller;
    std::cout << "PolarParallelMetaController created" << std::endl;

    // Linear dual PID meta controller
    cogip::motion_control::DualPIDMetaController linear_dualpid_meta_controller;
    polar_parallel_meta_controller.add_controller(&linear_dualpid_meta_controller);
    std::cout << "LinearDualPIDMetaController created and added to PolarParallelMetaController" << std::endl;
    // Linear pose PID controller
    cogip::pid::PID linear_position_pid(1, 0., 0., etl::numeric_limits<uint16_t>::max());
    cogip::motion_control::PosePIDControllerParameters linear_position_controller_parameters(&linear_position_pid);
    cogip::motion_control::PosePIDController linear_position_controller(&linear_position_controller_parameters);
    linear_dualpid_meta_controller.add_controller(&linear_position_controller);
    std::cout << "PosePIDController created and added to LinearDualPIDMetaController" << std::endl;
    // Linear speed filter
    cogip::motion_control::SpeedFilterParameters linear_speed_filter_parameters(1000., 500.);
    cogip::motion_control::SpeedFilter linear_speed_filter(&linear_speed_filter_parameters);
    linear_dualpid_meta_controller.add_controller(&linear_speed_filter);
    std::cout << "SpeedFilter created and added to LinearDualPIDMetaController" << std::endl;
    // Linear speed PID controller
    cogip::pid::PID linear_speed_pid(1, 0.1, 0., etl::numeric_limits<uint16_t>::max());
    cogip::motion_control::SpeedPIDControllerParameters linear_speed_controller_parameters(&linear_speed_pid);
    cogip::motion_control::SpeedPIDController linear_speed_controller(&linear_speed_controller_parameters);
    linear_dualpid_meta_controller.add_controller(&linear_speed_controller);
    std::cout << "SpeedPIDController created and added to LinearDualPIDMetaController" << std::endl;

    // Angular dual PID meta controller
    cogip::motion_control::DualPIDMetaController angular_dualpid_meta_controller;
    polar_parallel_meta_controller.add_controller(&angular_dualpid_meta_controller);
    std::cout << "AngularDualPIDMetaController created and added to PolarParallelMetaController" << std::endl;
    // Angular pose PID controller
    cogip::pid::PID angular_position_pid(1, 0., 0., etl::numeric_limits<uint16_t>::max());
    cogip::motion_control::PosePIDControllerParameters angular_position_controller_parameters(&angular_position_pid);
    cogip::motion_control::PosePIDController angular_position_controller(&angular_position_controller_parameters);
    angular_dualpid_meta_controller.add_controller(&angular_position_controller);
    std::cout << "PosePIDController created and added to AngularDualPIDMetaController" << std::endl;
    // Angular speed filter
    cogip::motion_control::SpeedFilterParameters angular_speed_filter_parameters(1000., 500.);
    cogip::motion_control::SpeedFilter angular_speed_filter(&angular_speed_filter_parameters);
    angular_dualpid_meta_controller.add_controller(&angular_speed_filter);
    std::cout << "SpeedFilter created and added to AngularDualPIDMetaController" << std::endl;
    // Angular speed PID controller
    cogip::pid::PID angular_speed_pid(1, 0.1, 0., etl::numeric_limits<uint16_t>::max());
    cogip::motion_control::SpeedPIDControllerParameters angular_speed_controller_parameters(&angular_speed_pid);
    cogip::motion_control::SpeedPIDController angular_speed_controller(&angular_speed_controller_parameters);
    angular_dualpid_meta_controller.add_controller(&angular_speed_controller);
    std::cout << "SpeedPIDController created and added to AngularDualPIDMetaController" << std::endl;

    // Quad PID meta controller
    cogip::motion_control::QuadPIDMetaController quadpid_meta_controller;
    quadpid_meta_controller.add_controller(&pose_straight_filter);
    quadpid_meta_controller.add_controller(&polar_parallel_meta_controller);

    engine.set_controller(&quadpid_meta_controller);

    puts("\n== Start thread with current controller ==");
    engine.start_thread();

    char line_buf[SHELL_DEFAULT_BUFSIZE];
    shell_run(NULL, line_buf, SHELL_DEFAULT_BUFSIZE);

    exit(0);
}
