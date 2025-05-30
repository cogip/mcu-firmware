// RIOT includes
#include <shell.h>
#include <ztimer.h>

// System includes
#include <cstdio>
#include <iostream>

#include "etl/list.h"
#include "etl/vector.h"

#include "pid/PID.hpp"
#include "board.h"
#include "drive_controller/DifferentialDriveControllerParameters.hpp"
#include "drive_controller/DifferentialDriveController.hpp"
#include "encoder/EncoderQDEC.hpp"
#include "localization/LocalizationDifferential.hpp"
#include "motor/MotorDriverDRV8873.hpp"
#include "motor/MotorRIOT.hpp"
#include "motion_control_common/Controller.hpp"
#include "dualpid_meta_controller/DualPIDMetaController.hpp"
#include "quadpid_meta_controller/QuadPIDMetaController.hpp"
#include "platform_engine/PlatformEngine.hpp"
#include "pose_pid_controller/PosePIDController.hpp"
#include "speed_pid_controller/SpeedPIDController.hpp"
#include "pose_straight_filter/PoseStraightFilter.hpp"
#include "speed_filter/SpeedFilter.hpp"
#include "polar_parallel_meta_controller/PolarParallelMetaController.hpp"

/// Encoders
static cogip::encoder::EncoderQDEC left_encoder(0, cogip::encoder::EncoderMode::ENCODER_MODE_X1, 1024);
static cogip::encoder::EncoderQDEC right_encoder(1, cogip::encoder::EncoderMode::ENCODER_MODE_X1, 1024);


/// Odometry
static cogip::localization::LocalizationDifferentialParameters localization_params(
    50.00,
    50.00,
    100.00,
    -1,
    1);
static cogip::localization::LocalizationDifferential localization(localization_params, left_encoder, right_encoder);

/// Motor driver
static const motor_driver_params_t motion_motors_params = {
    .mode = MOTOR_DRIVER_1_DIR_BRAKE,
    .pwm_dev = 0,
    .pwm_mode = PWM_LEFT,
    .pwm_frequency = 20000U,
    .pwm_resolution = 500U,
    .brake_inverted = true,
    .enable_inverted = false,
    .nb_motors = 2,
    .motors = {
        // Left motor
        {
            .pwm_channel = 0,
            .gpio_enable = GPIO_PIN(PORT_A, 10),
            .gpio_dir0 = GPIO_PIN(PORT_C, 6),
            .gpio_brake = GPIO_PIN(PORT_C, 8),
            .gpio_dir_reverse = 1,
        },
        // Right motor
        {
            .pwm_channel = 1,
            .gpio_enable = GPIO_PIN(PORT_B, 1),
            .gpio_dir0 = GPIO_PIN(PORT_B, 10),
            .gpio_brake = GPIO_PIN(PORT_B, 2),
            .gpio_dir_reverse = 0,
        },
    },
};

static cogip::motor::MotorDriverDRV8873 motor_driver(motion_motors_params);

/// Motors
static cogip::motor::MotorRIOT left_motor(motor_driver, 0);
static cogip::motor::MotorRIOT right_motor(motor_driver, 1);

static cogip::drive_controller::DifferentialDriveControllerParameters drive_controller_params(
    50.00,
    50.00,
    100.0,
    1.0,
    1.0,
    0.0,
    100.0,
    20);

static cogip::drive_controller::DifferentialDriveController drive_controller(drive_controller_params, left_motor, right_motor);

static void pf_pose_reached_cb([[maybe_unused]] const cogip::motion_control::target_pose_status_t state)
{

}

static constexpr uint32_t motion_control_thread_period_ms = 20;

// Motion control engine
static cogip::motion_control::PlatformEngine motion_control_platform_engine(localization,
    drive_controller,
    cogip::motion_control::pose_reached_cb_t::create<pf_pose_reached_cb>(),
    motion_control_thread_period_ms
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

    motion_control_platform_engine.set_controller(&quadpid_meta_controller);

    puts("\n== Start thread with current controller ==");
    motion_control_platform_engine.start_thread();

    char line_buf[SHELL_DEFAULT_BUFSIZE];
    shell_run(NULL, line_buf, SHELL_DEFAULT_BUFSIZE);

    exit(0);
}
