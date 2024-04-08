// Project includes
#include "motor_driver.h"

namespace cogip {

namespace pf {

namespace actuators {

/// Motion control motors
static const motor_driver_params_t actuators_motors_params =
{
    .mode = MOTOR_DRIVER_1_DIR,
    .pwm_dev = 0,
    .pwm_mode = PWM_LEFT,
    .pwm_frequency = 20000U,
    .pwm_resolution = 100U,
    .brake_inverted = true,
    .enable_inverted = false,
    .nb_motors = 2,
    .motors = {
        // Lift motor
        {
            .pwm_channel = 2,
            .gpio_enable = GPIO_PIN(PORT_A, 8),
            .gpio_dir0 = GPIO_PIN(PORT_A, 11),
            .gpio_dir_reverse = 0,
        },
        // Ball launcher (no direction, no enable, just PWM) & conveyor (no PWM, no direction, just the enable)
        {
            .pwm_channel = 1,
            .gpio_enable = GPIO_PIN(PORT_B, 1),
            .gpio_dir0 = GPIO_UNDEF,
            .gpio_dir_reverse = 0,
        },
    },
    .motor_set_post_cb = nullptr
};

} // namespace actuators

} // namespace pf

} // namespace cogip
