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

/*
 * Motor brake pins are board hardware: the J8/J9 connector pinout was
 * reassigned between the G474 module (cogip-board) and the STM32H563 module
 * (cogip-board-h5), so a brake can land on a different MCU pin per board. The
 * defaults below match cogip-board (G4); a board that routes a brake elsewhere
 * overrides the matching macro in its own board.h (included above), e.g.
 * cogip-board-h5 defines MOTION_MOTOR0_BRAKE_PIN = PB3.
 */
#ifndef MOTION_MOTOR0_BRAKE_PIN
#define MOTION_MOTOR0_BRAKE_PIN GPIO_PIN(PORT_C, 8)
#endif
#ifndef MOTION_MOTOR1_BRAKE_PIN
#define MOTION_MOTOR1_BRAKE_PIN GPIO_PIN(PORT_B, 2)
#endif

/**
 * @brief Simulate QDEC on motor_set() calls
 *
 * @param[in] motor_driver      motor driver to which motor is attached
 * @param[in] motor_id          motor ID on driver
 * @param[in] pwm_duty_cycle    Signed PWM duty_cycle to set motor speed and
 * direction
 *
 * @return                      0 on success
 */
void cogip_native_motor_driver_qdec_simulation(const motor_driver_t* motor_driver, uint8_t motor_id,
                                               int32_t pwm_duty_cycle);

/// Motion control motors
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
                .gpio_brake = MOTION_MOTOR0_BRAKE_PIN,
                .gpio_dir_reverse = 1,
            },
            // Right motor
            {
                .pwm_channel = 1,
                .gpio_enable = GPIO_PIN(PORT_B, 1),
                .gpio_dir0 = GPIO_PIN(PORT_B, 10),
                .gpio_brake = MOTION_MOTOR1_BRAKE_PIN,
                .gpio_dir_reverse = 0,
            },
        },
    .motor_set_post_cb = MOTION_MOTORS_POST_CB};

} // namespace motion_control

} // namespace pf

} // namespace cogip
