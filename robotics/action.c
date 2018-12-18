#include "action.h"

#include <stdint.h>
#include "periph/gpio.h"
#include "xtimer.h"
#include "platform.h"

#include "actuators/motor_pap.h"
#include "actuators/sd21.h"


/* Nb water balls in a pipe */
#define PIPE_WATER_NB   8

/* 500ms between valves positions */
#define DELAY_VALVE_US  (500UL * US_PER_MS)

/* 500ms between 2 storages positions of the wheel */
#define DELAY_BETWEEN_STORAGE_US    (500UL * US_PER_MS)

/* 100ms for launcher to start */
#define DELAY_LAUNCHER_US   (100UL * US_PER_MS)

/* 300 to let the recycler to push the ball */
#define DELAY_RECYCLER_US   (300UL * US_PER_MS)

/* Number of valve per ball (multiple trials in case ball are blocked ?) */
/* Set 1 if confirmed Ok !! */
#define VALVE_TRIAL_NB  1

/*
 * private
 */

static void _ball_launcher_motor_enable(uint8_t enable)
{
    gpio_write(GPIO_PIN(PORT_B, 15), enable);
}

/*
 * public
 */

void action_init(void)
{
    // initial positions in case of transient reboot
    gpio_init(GPIO_PIN(PORT_B, 15), GPIO_OUT); // FIXME: put elsewhere
}

// before: wheel empty
// after:  wheel full
void act_catch_same_color_water(void)
{
    for (int i = 0; i < PIPE_WATER_NB /*-1*/; i++) {
        motor_pap_turn_next_storage();
        xtimer_usleep(DELAY_BETWEEN_STORAGE_US);
    }
}

// before: wheel full
// after:  wheel empty
void act_launch_same_color_water(void)
{
    for (int i = 0; i < PIPE_WATER_NB; i++) {

        _ball_launcher_motor_enable(TRUE);

        // Wait launcher to start
        xtimer_usleep(DELAY_LAUNCHER_US);

        // Two trials in case ball is stuck?
        for (uint8_t j = 0; j < VALVE_TRIAL_NB; j++) {
            sd21_control_servo(&sd21, SERVO_ID_VALVE_LAUNCHER, SD21_SERVO_OPEN);

            // wait 500ms
            xtimer_usleep(DELAY_VALVE_US);

            sd21_control_servo(&sd21, SERVO_ID_VALVE_LAUNCHER, SD21_SERVO_CLOSE);

            // wait 500ms (on first trial only)
            xtimer_usleep(DELAY_VALVE_US);
        }

        _ball_launcher_motor_enable(FALSE);

        // turn the wheel, by 1 step (excluding the last storage)
        if (i < PIPE_WATER_NB - 1) {
            motor_pap_turn_next_storage();
            xtimer_usleep(DELAY_BETWEEN_STORAGE_US);
        }
    }
}

// wheel entry point is 2 storage before valve's one, thus same color

// before: wheel empty
// after:  wheel full
void act_catch_mixed_water(void)
{
    for (int i = 0; i < PIPE_WATER_NB; i++) {
        motor_pap_turn_next_storage();
        xtimer_usleep(DELAY_BETWEEN_STORAGE_US);
    }
}

// before: wheel full
// after:  wheel half, lefting opponent color
void act_launch_mixed_water(void)
{
    for (int i = 0; i < PIPE_WATER_NB; i += 2) {

        _ball_launcher_motor_enable(TRUE);

        // Wait launcher to start
        xtimer_usleep(DELAY_LAUNCHER_US);

        // Two trials in case ball is stuck?
        for (uint8_t j = 0; j < VALVE_TRIAL_NB; j++) {
            sd21_control_servo(&sd21, SERVO_ID_VALVE_LAUNCHER, SD21_SERVO_OPEN);

            // wait 500ms
            xtimer_usleep(DELAY_VALVE_US);

            sd21_control_servo(&sd21, SERVO_ID_VALVE_LAUNCHER, SD21_SERVO_CLOSE);

            // wait 500ms (on first trial only)
            xtimer_usleep(DELAY_VALVE_US);
        }

        _ball_launcher_motor_enable(FALSE);

        // turn the wheel, by 2 steps (including the last storage)
        if (i < PIPE_WATER_NB /*-1*/) {
            motor_pap_turn_next_storage();
            motor_pap_turn_next_storage();
            xtimer_usleep(DELAY_BETWEEN_STORAGE_US);
        }
    }
    motor_pap_turn_next_storage();

}

// before: wheel half, lefting opponent color
// after:  wheel empty
void act_drop_recycled_water(void)
{
    for (int i = 0; i < PIPE_WATER_NB; i += 2) {

        /* Open valve */
        sd21_control_servo(&sd21, SERVO_ID_VALVE_RECYCLER, SD21_SERVO_OPEN);
        xtimer_usleep(DELAY_VALVE_US);

        /* Push the ball */
        sd21_control_servo(&sd21, SERVO_ID_RECYCLER, SD21_SERVO_CLOSE);
        xtimer_usleep(DELAY_RECYCLER_US);

        sd21_control_servo(&sd21, SERVO_ID_RECYCLER, SD21_SERVO_OPEN);
        //xtimer_usleep(DELAY_RECYCLER_US);

        /* Close valve */
        sd21_control_servo(&sd21, SERVO_ID_VALVE_RECYCLER, SD21_SERVO_CLOSE);
        xtimer_usleep(DELAY_VALVE_US);

        // turn the wheel, by 2 steps (except for the last storage)
        if (i < PIPE_WATER_NB - 1) {
            motor_pap_turn_next_storage();
            motor_pap_turn_next_storage();
            xtimer_usleep(DELAY_BETWEEN_STORAGE_US);
        }
    }
}

void act_open_bee_pusher(void)
{
    if (pf_is_camp_left()) {
        sd21_control_servo(&sd21, SERVO_ID_BEE_L, SD21_SERVO_OPEN);
    }
    else {
        sd21_control_servo(&sd21, SERVO_ID_BEE_R, SD21_SERVO_OPEN);
    }
}

void act_close_bee_pusher(void)
{
    if (pf_is_camp_left()) {
        sd21_control_servo(&sd21, SERVO_ID_BEE_L, SD21_SERVO_CLOSE);
    }
    else {
        sd21_control_servo(&sd21, SERVO_ID_BEE_R, SD21_SERVO_CLOSE);
    }
}
