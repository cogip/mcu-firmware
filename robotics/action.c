#include "action.h"

#include <stdint.h>
#include "periph/gpio.h"
#include "xtimer.h"
#include "platform.h"
#include "actuators/sd21.h"

#include "actuators/motor_pap.h"

/* Nb water balls in a pipe */
#define PIPE_WATER_NB			8

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
	for (int i = 0; i < PIPE_WATER_NB/*-1*/; i++)
		motor_pap_turn_next_storage();
}

// before: wheel full
// after:  wheel empty
void act_launch_same_color_water(void)
{
	for (int i = 0; i < PIPE_WATER_NB; i++) {

		sd21_control_servo(&sd21, SERVO_ID_VALVE_RECYCLER, SD21_SERVO_OPEN);

		// wait 500ms
		xtimer_usleep(500*1000/* US */);

		sd21_control_servo(&sd21, SERVO_ID_RECYCLER, SD21_SERVO_CLOSE);
		// wait 300ms
		xtimer_usleep(300*1000/* US */);

		sd21_control_servo(&sd21, SERVO_ID_RECYCLER, SD21_SERVO_OPEN);
		/// wait 300ms
		//xtimer_usleep(300*1000/* US */);

		sd21_control_servo(&sd21, SERVO_ID_VALVE_RECYCLER, SD21_SERVO_CLOSE);

		// wait 500ms (on first trial only)
		xtimer_usleep(500*1000/* US */);

		// turn the wheel, by 2 steps (except for the last storage)
		if (i < PIPE_WATER_NB-1) {
			motor_pap_turn_next_storage();
			motor_pap_turn_next_storage();
		}
	}

}

// wheel entry point is 2 storage before valve's one, thus same color

// before: wheel empty
// after:  wheel full
void act_catch_interleaved_water(void)
{
	for (int i = 0; i < PIPE_WATER_NB; i++)
		motor_pap_turn_next_storage();
}

// before: wheel full
// after:  wheel half, lefting opponent color
void act_launch_interleaved_water(void)
{
	for (int i = 0; i < PIPE_WATER_NB; i++) {

		_ball_launcher_motor_enable(TRUE);

		// wait 100ms
		xtimer_usleep(100*1000/* US */);

		// Two trials in case ball is stuck?
		for (uint8_t j = 0; j < 2; j++) {
			sd21_control_servo(&sd21, SERVO_ID_VALVE_LAUNCHER, SD21_SERVO_OPEN);

			// wait 500ms
			xtimer_usleep(500*1000/* US */);

			sd21_control_servo(&sd21, SERVO_ID_VALVE_LAUNCHER, SD21_SERVO_CLOSE);

			// wait 500ms (on first trial only)
			if (!j)
				xtimer_usleep(500*1000/* US */);
		}

		_ball_launcher_motor_enable(FALSE);

		// turn the wheel, by 2 steps (including the last storage)
		if (i < PIPE_WATER_NB/*-1*/) {
			motor_pap_turn_next_storage();
			motor_pap_turn_next_storage();
		}
	}

}

// before: wheel half, lefting opponent color
// after:  wheel empty
void act_drop_recycled_water(void)
{
	for (int i = 0; i < PIPE_WATER_NB; i++) {

		//_ball_launcher_motor_enable(TRUE);

		// wait 500ms
		//xtimer_usleep(100*1000/* US */);

		// Two trials in case ball is stuck?
		for (uint8_t j = 0; j < 2; j++) {
			sd21_control_servo(&sd21, SERVO_ID_VALVE_LAUNCHER, SD21_SERVO_OPEN);

			// wait 500ms
			xtimer_usleep(500*1000/* US */);

			sd21_control_servo(&sd21, SERVO_ID_VALVE_LAUNCHER, SD21_SERVO_CLOSE);

			// wait 500ms (on first trial only)
			if (!j)
				xtimer_usleep(500*1000/* US */);
		}

		_ball_launcher_motor_enable(FALSE);

		// turn the wheel, by 2 steps (excluding the last storage)
		if (i < PIPE_WATER_NB-1) {
			motor_pap_turn_next_storage();
			motor_pap_turn_next_storage();
		}
	}

}

void act_open_bee_pusher(void)
{
	if (mach_is_camp_yellow())
		sd21_control_servo(&sd21, SERVO_ID_BEE_L, SD21_SERVO_OPEN);
	else
		sd21_control_servo(&sd21, SERVO_ID_BEE_R, SD21_SERVO_OPEN);
}

void act_close_bee_pusher(void)
{
	if (mach_is_camp_yellow())
		sd21_control_servo(&sd21, SERVO_ID_BEE_L, SD21_SERVO_CLOSE);
	else
		sd21_control_servo(&sd21, SERVO_ID_BEE_R, SD21_SERVO_CLOSE);
}
