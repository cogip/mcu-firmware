// Project includes
#pragma once

#include "board.h"
#include "motor_driver.h"

namespace cogip {
namespace app {
namespace actuators {

#define CLEAR_OVERLOAD_PIN  GPIO_PIN(PORT_C, 13)

/// Motion control motors
static const motor_driver_params_t actuators_motors_params =
{
    .mode = MOTOR_DRIVER_1_DIR_BRAKE,
    .pwm_dev = 0,
    .pwm_mode = PWM_LEFT,
    .pwm_frequency = 20000U,
    .pwm_resolution = 500U,
    .brake_inverted = true,
    .enable_inverted = false,
    .nb_motors = 1,
    .motors = {
        // Bottom lift motor
        {
            .pwm_channel = 0,
            .gpio_enable = GPIO_PIN(PORT_A, 10),
            .gpio_dir0 = GPIO_PIN(PORT_C, 6),
            .gpio_brake = GPIO_PIN(PORT_C, 8),
            .gpio_dir_reverse = 1,
        },
    },
    .motor_set_post_cb = nullptr
};

} // namespace actuators
} // namespace app
} // namespace cogip
