// Project includes
#include "board.h"
#include "motor_driver.h"

namespace cogip {

namespace pf {

namespace actuators {

/// Motion control motors
static const motor_driver_params_t motion_motors_params =
{
    .mode = MOTOR_DRIVER_1_DIR_BRAKE,
    .pwm_dev = 0,
    .pwm_mode = PWM_LEFT,
    .pwm_frequency = 20000U,
    .pwm_resolution = 500U,
    .brake_inverted = true,
    .enable_inverted = false,
    .nb_motors = 2,
    .motors = {
        // Bottom lift motor
        {
            .pwm_channel = 0,
            .gpio_enable = GPIO_PIN(PORT_A, 10),
            .gpio_dir0 = GPIO_PIN(PORT_C, 6),
            .gpio_brake = GPIO_PIN(PORT_C, 8),
            .gpio_dir_reverse = 0,
        },
        // Top lift motor
        {
            .pwm_channel = 1,
            .gpio_enable = GPIO_PIN(PORT_B, 1),
            .gpio_dir0 = GPIO_PIN(PORT_B, 10),
            .gpio_brake = GPIO_PIN(PORT_B, 2),
            .gpio_dir_reverse = 0,
        },
    },
    .motor_set_post_cb = nullptr
};

} // namespace actuators

} // namespace pf

} // namespace cogip
