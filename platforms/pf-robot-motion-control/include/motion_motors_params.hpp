// Project includes
#include "board.h"
#include "motor_driver.h"

namespace cogip {

namespace pf {

namespace motion_control {

/* Motion motors */
#define MOTOR_LEFT  0
#define MOTOR_RIGHT 1

/* Quadrature decoding polarity */
#define QDEC_LEFT_POLARITY  -1
#define QDEC_RIGHT_POLARITY 1

#ifndef MOTION_MOTORS_POST_CB
/// Motion control callback on motor_set() call
#define MOTION_MOTORS_POST_CB nullptr
#endif

/// Quadrature decoding
#ifndef QDEC_MODE
#define QDEC_MODE QDEC_X4
#endif

/**
 * @brief Simulate QDEC on motor_set() calls
 *
 * @param[in] motor_driver      motor driver to which motor is attached
 * @param[in] motor_id          motor ID on driver
 * @param[in] pwm_duty_cycle    Signed PWM duty_cycle to set motor speed and direction
 *
 * @return                      0 on success
 */
void cogip_native_motor_driver_qdec_simulation(
    const motor_driver_t *motor_driver, uint8_t motor_id,
    int32_t pwm_duty_cycle);

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
    .motor_set_post_cb = MOTION_MOTORS_POST_CB
};

} // namespace motion_control

} // namespace pf

} // namespace cogip
