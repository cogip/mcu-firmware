// Project includes
#include "board.h"
#include "motor_driver.h"

namespace cogip {

namespace pf {

namespace motion_control {

#ifndef MOTION_MOTORS_POST_CB
/// Motion control callback on motor_set() call
#define MOTION_MOTORS_POST_CB nullptr
#endif

/// Motion control motors
static const motor_driver_params_t motion_motors_params =
{
    .mode = MOTOR_DRIVER_1_DIR_BRAKE,
    .pwm_dev = 0,
    .pwm_mode = PWM_LEFT,
    .pwm_frequency = 20000U,
    .pwm_resolution = 1000U,
    .brake_level = MOTOR_BRAKE_HIGH,
    .enable_level = MOTOR_ENABLE_HIGH,
    .nb_motors = 2,
    .motors = {
        // Left motor
        {
            .pwm_channel = 0,
            .gpio_enable = GPIO_PIN(PORT_B, 10),
            .gpio_dir0 = GPIO_PIN(PORT_B, 2),
            .gpio_dir1_or_brake = GPIO_PIN(PORT_B, 12),
            .gpio_dir_reverse = 1,
        },
        // Right motor
        {
            .pwm_channel = 1,
            .gpio_enable = GPIO_PIN(PORT_B, 10),
            .gpio_dir0 = GPIO_PIN(PORT_B, 0),
            .gpio_dir1_or_brake = GPIO_PIN(PORT_C, 4),
            .gpio_dir_reverse = 0,
        },
    },
    .motor_set_post_cb = MOTION_MOTORS_POST_CB
};

} // namespace motion_control

} // namespace pf

} // namespace cogip
