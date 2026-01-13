// RIOT includes
#include <cstddef>
#include <shell.h>
#include <ztimer.h>

// System includes
#include "log.h"
#include <cstdio>

#include "etl/list.h"
#include "etl/vector.h"

#include "board.h"
#include "drive_controller/DifferentialDriveController.hpp"
#include "drive_controller/DifferentialDriveControllerParameters.hpp"
#include "dualpid_meta_controller/DualPIDMetaController.hpp"
#include "encoder/EncoderQDEC.hpp"
#include "localization/LocalizationDifferential.hpp"
#include "motion_control_common/Controller.hpp"
#include "motor/MotorDriverDRV8873.hpp"
#include "motor/MotorRIOT.hpp"
#include "parameter/Parameter.hpp"
#include "pid/PID.hpp"
#include "pid/PIDParameters.hpp"
#include "platform_engine/PlatformEngine.hpp"
#include "polar_parallel_meta_controller/PolarParallelMetaController.hpp"
#include "pose_pid_controller/PosePIDController.hpp"
#include "pose_pid_controller/PosePIDControllerIOKeysDefault.hpp"
#include "pose_straight_filter/PoseStraightFilter.hpp"
#include "pose_straight_filter/PoseStraightFilterIOKeysDefault.hpp"
#include "quadpid_meta_controller/QuadPIDMetaController.hpp"
#include "speed_filter/SpeedFilter.hpp"
#include "speed_filter/SpeedFilterIOKeysDefault.hpp"
#include "speed_pid_controller/SpeedPIDController.hpp"
#include "speed_pid_controller/SpeedPIDControllerIOKeysDefault.hpp"

/// Odometry parameters
static cogip::parameter::Parameter<float> left_encoder_wheels_diameter_mm{50.00};
static cogip::parameter::Parameter<float> right_encoder_wheels_diameter_mm{50.00};
static cogip::parameter::Parameter<float> encoder_wheels_distance_mm{100.00};
static cogip::parameter::Parameter<float> qdec_left_polarity{-1.0};
static cogip::parameter::Parameter<float> qdec_right_polarity{1.0};

/// Encoders
static cogip::encoder::EncoderQDEC left_encoder(0, cogip::encoder::EncoderMode::ENCODER_MODE_X1,
                                                1024);
static cogip::encoder::EncoderQDEC right_encoder(1, cogip::encoder::EncoderMode::ENCODER_MODE_X1,
                                                 1024);

/// Odometry
static cogip::localization::LocalizationDifferentialParameters
    localization_params(left_encoder_wheels_diameter_mm, right_encoder_wheels_diameter_mm,
                        encoder_wheels_distance_mm, qdec_left_polarity, qdec_right_polarity);
static cogip::localization::LocalizationDifferential localization(localization_params, left_encoder,
                                                                  right_encoder);

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
    .motors =
        {
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

static cogip::drive_controller::DifferentialDriveControllerParameters
    drive_controller_params(50.00, 50.00, 100.0, 1.0, 1.0, 0.0, 100.0, 20);

static cogip::drive_controller::DifferentialDriveController
    drive_controller(drive_controller_params, left_motor, right_motor);

static void
pf_pose_reached_cb([[maybe_unused]] const cogip::motion_control::target_pose_status_t state)
{
}

static constexpr uint32_t motion_control_thread_period_ms = 20;

// Motion control engine
static cogip::motion_control::PlatformEngine motion_control_platform_engine(
    localization, drive_controller,
    cogip::motion_control::pose_reached_cb_t::create<pf_pose_reached_cb>(),
    motion_control_thread_period_ms);

int main(void)
{
    LOG_INFO("== Controller skeleton ==\n");

    // Filter controller behavior to always moves in a straight line
    cogip::motion_control::PoseStraightFilterParameters pose_straight_filter_parameters =
        cogip::motion_control::PoseStraightFilterParameters(2, 2);
    cogip::motion_control::PoseStraightFilter pose_straight_filter =
        cogip::motion_control::PoseStraightFilter(
            cogip::motion_control::pose_straight_filter_io_keys_default,
            pose_straight_filter_parameters);
    LOG_INFO("PoseStraightFilter created\n");

    // Split angular and linear controls
    cogip::motion_control::PolarParallelMetaController polar_parallel_meta_controller;
    LOG_INFO("PolarParallelMetaController created\n");

    // Linear dual PID meta controller
    cogip::motion_control::DualPIDMetaController linear_dualpid_meta_controller;
    polar_parallel_meta_controller.add_controller(&linear_dualpid_meta_controller);
    LOG_INFO("LinearDualPIDMetaController created and added to "
             "PolarParallelMetaController\n");
    // Linear pose PID controller
    static cogip::parameter::Parameter<float, cogip::parameter::NonNegative> linear_pos_kp{1.f};
    static cogip::parameter::Parameter<float, cogip::parameter::NonNegative> linear_pos_ki{0.f};
    static cogip::parameter::Parameter<float, cogip::parameter::NonNegative> linear_pos_kd{0.f};
    static cogip::parameter::Parameter<float, cogip::parameter::NonNegative> linear_pos_limit{
        static_cast<float>(etl::numeric_limits<uint16_t>::max())};
    cogip::pid::PIDParameters linear_position_pid_params(linear_pos_kp, linear_pos_ki,
                                                         linear_pos_kd, linear_pos_limit);
    cogip::pid::PID linear_position_pid(linear_position_pid_params);
    cogip::motion_control::PosePIDControllerParameters linear_position_controller_parameters(
        &linear_position_pid);
    cogip::motion_control::PosePIDController linear_position_controller(
        cogip::motion_control::linear_pose_pid_controller_io_keys_default,
        linear_position_controller_parameters);
    linear_dualpid_meta_controller.add_controller(&linear_position_controller);
    LOG_INFO("PosePIDController created and added to LinearDualPIDMetaController\n");
    // Linear speed filter
    cogip::motion_control::SpeedFilterParameters linear_speed_filter_parameters(1000., 500.);
    cogip::motion_control::SpeedFilter linear_speed_filter(
        cogip::motion_control::linear_speed_filter_io_keys_default, linear_speed_filter_parameters);
    linear_dualpid_meta_controller.add_controller(&linear_speed_filter);
    LOG_INFO("SpeedFilter created and added to LinearDualPIDMetaController\n");
    // Linear speed PID controller
    static cogip::parameter::Parameter<float, cogip::parameter::NonNegative> linear_spd_kp{1.f};
    static cogip::parameter::Parameter<float, cogip::parameter::NonNegative> linear_spd_ki{0.1f};
    static cogip::parameter::Parameter<float, cogip::parameter::NonNegative> linear_spd_kd{0.f};
    static cogip::parameter::Parameter<float, cogip::parameter::NonNegative> linear_spd_limit{
        static_cast<float>(etl::numeric_limits<uint16_t>::max())};
    cogip::pid::PIDParameters linear_speed_pid_params(linear_spd_kp, linear_spd_ki, linear_spd_kd,
                                                      linear_spd_limit);
    cogip::pid::PID linear_speed_pid(linear_speed_pid_params);
    cogip::motion_control::SpeedPIDControllerParameters linear_speed_controller_parameters(
        &linear_speed_pid);
    cogip::motion_control::SpeedPIDController linear_speed_controller(
        cogip::motion_control::linear_speed_pid_controller_io_keys_default,
        linear_speed_controller_parameters);
    linear_dualpid_meta_controller.add_controller(&linear_speed_controller);
    LOG_INFO("SpeedPIDController created and added to LinearDualPIDMetaController\n");

    // Angular dual PID meta controller
    cogip::motion_control::DualPIDMetaController angular_dualpid_meta_controller;
    polar_parallel_meta_controller.add_controller(&angular_dualpid_meta_controller);
    LOG_INFO("AngularDualPIDMetaController created and added to "
             "PolarParallelMetaController\n");
    // Angular pose PID controller
    static cogip::parameter::Parameter<float, cogip::parameter::NonNegative> angular_pos_kp{1.f};
    static cogip::parameter::Parameter<float, cogip::parameter::NonNegative> angular_pos_ki{0.f};
    static cogip::parameter::Parameter<float, cogip::parameter::NonNegative> angular_pos_kd{0.f};
    static cogip::parameter::Parameter<float, cogip::parameter::NonNegative> angular_pos_limit{
        static_cast<float>(etl::numeric_limits<uint16_t>::max())};
    cogip::pid::PIDParameters angular_position_pid_params(angular_pos_kp, angular_pos_ki,
                                                          angular_pos_kd, angular_pos_limit);
    cogip::pid::PID angular_position_pid(angular_position_pid_params);
    cogip::motion_control::PosePIDControllerParameters angular_position_controller_parameters(
        &angular_position_pid);
    cogip::motion_control::PosePIDController angular_position_controller(
        cogip::motion_control::angular_pose_pid_controller_io_keys_default,
        angular_position_controller_parameters);
    angular_dualpid_meta_controller.add_controller(&angular_position_controller);
    LOG_INFO("PosePIDController created and added to AngularDualPIDMetaController\n");
    // Angular speed filter
    cogip::motion_control::SpeedFilterParameters angular_speed_filter_parameters(1000., 500.);
    cogip::motion_control::SpeedFilter angular_speed_filter(
        cogip::motion_control::angular_speed_filter_io_keys_default,
        angular_speed_filter_parameters);
    angular_dualpid_meta_controller.add_controller(&angular_speed_filter);
    LOG_INFO("SpeedFilter created and added to AngularDualPIDMetaController\n");
    // Angular speed PID controller
    static cogip::parameter::Parameter<float, cogip::parameter::NonNegative> angular_spd_kp{1.f};
    static cogip::parameter::Parameter<float, cogip::parameter::NonNegative> angular_spd_ki{0.1f};
    static cogip::parameter::Parameter<float, cogip::parameter::NonNegative> angular_spd_kd{0.f};
    static cogip::parameter::Parameter<float, cogip::parameter::NonNegative> angular_spd_limit{
        static_cast<float>(etl::numeric_limits<uint16_t>::max())};
    cogip::pid::PIDParameters angular_speed_pid_params(angular_spd_kp, angular_spd_ki,
                                                       angular_spd_kd, angular_spd_limit);
    cogip::pid::PID angular_speed_pid(angular_speed_pid_params);
    cogip::motion_control::SpeedPIDControllerParameters angular_speed_controller_parameters(
        &angular_speed_pid);
    cogip::motion_control::SpeedPIDController angular_speed_controller(
        cogip::motion_control::angular_speed_pid_controller_io_keys_default,
        angular_speed_controller_parameters);
    angular_dualpid_meta_controller.add_controller(&angular_speed_controller);
    LOG_INFO("SpeedPIDController created and added to AngularDualPIDMetaController\n");

    // Quad PID meta controller
    cogip::motion_control::QuadPIDMetaController quadpid_meta_controller;
    quadpid_meta_controller.add_controller(&pose_straight_filter);
    quadpid_meta_controller.add_controller(&polar_parallel_meta_controller);

    motion_control_platform_engine.set_controller(&quadpid_meta_controller);

    LOG_INFO("== Start thread with current controller ==\n");
    motion_control_platform_engine.start_thread();

    char line_buf[SHELL_DEFAULT_BUFSIZE];
    shell_run(NULL, line_buf, SHELL_DEFAULT_BUFSIZE);

    exit(0);
}
