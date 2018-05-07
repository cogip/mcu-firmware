#include "motor_pap.h"

#include "periph/gpio.h"
#include "xtimer.h"

/*
 * Pin mux definition
 */

#define GPIO_STEPPER_EN		GPIO_PIN(PORT_C, 11)
#define GPIO_STEPPER_DIR	GPIO_PIN(PORT_A, 10)
#define GPIO_STEPPER_STEP	GPIO_PIN(PORT_A, 11)

#define GPIO_SHARP_SENSOR1	GPIO_PIN(PORT_A, 7)

#define HALF_PERIOD_WAIT_US	(10UL * US_PER_MS)

#define STEP_MAX_BEFORE_BLOCKED	50

/* If following is defined, use sharp measurement on each stepper motor steps */
#define MEASUREMENT_ON_EACH_STEP
/* Number of step we consider the Sharp input value for end of turn */
#define MEASUREMENT_GRANULARITY	10

/*
 * internal helpers functions
 */

/* stepper controller */
static inline void _stepper_en(uint8_t en)
{
	gpio_write(GPIO_STEPPER_EN, !en); /* we do the logic inversion here */
}

static inline void _stepper_dir_set_cw(void)
{
	gpio_write(GPIO_STEPPER_DIR, 1);
}

static inline void _stepper_dir_set_ccw(void)
{
	gpio_write(GPIO_STEPPER_DIR, 0);
}

/* wheel detection */
static inline uint8_t _sharp_detects_hole(void)
{
	return gpio_read(GPIO_SHARP_SENSOR1);
}

/* some sequences */
static inline void _bitbanging_do_one_step(void)
{
	gpio_write(GPIO_STEPPER_STEP, 1);
	xtimer_usleep(HALF_PERIOD_WAIT_US);

	gpio_write(GPIO_STEPPER_STEP, 0);
	xtimer_usleep(HALF_PERIOD_WAIT_US);
}

#if !defined(MEASUREMENT_ON_EACH_STEP)
static void _turn_unconditionally(uint16_t nb_steps)
{
	for (uint16_t steps = 0; steps < nb_steps; steps ++)
		_bitbanging_do_one_step();
}
#endif

/*
 * Public functions
 */

void motor_pap_init(void)
{
	gpio_init(GPIO_STEPPER_EN, GPIO_OUT);
	gpio_init(GPIO_STEPPER_DIR, GPIO_OUT);
	gpio_init(GPIO_STEPPER_STEP, GPIO_OUT);

	_stepper_en(FALSE);
}

uint8_t motor_pap_turn_next_storage(void)
{
	uint8_t i = 0;
	uint8_t retval = 0;
	uint8_t sharp_initial = FALSE;

	_stepper_en(TRUE);
	_stepper_dir_set_ccw();

	sharp_initial = _sharp_detects_hole();

	/* First ensure the value is logic 1 */
	if (! sharp_initial) {
		for (i = 0; i < STEP_MAX_BEFORE_BLOCKED; i++) {
#if defined(MEASUREMENT_ON_EACH_STEP)
			_bitbanging_do_one_step();
#else
			_turn_unconditionally(MEASUREMENT_GRANULARITY);
#endif

			printf("[%03d]\tTOR = %d\n", i, _sharp_detects_hole());

			if (_sharp_detects_hole() != sharp_initial)
				break;
		}
	}

	/* PAP is blocked, mechanical issue, should invalidate all actions ! */
	if (i == STEP_MAX_BEFORE_BLOCKED) {
		retval = 1;
		goto exit_point;
	}

	sharp_initial = _sharp_detects_hole();

	/* move till next storage location */
	for (i = 0; i < STEP_MAX_BEFORE_BLOCKED; i++) {
#if defined(MEASUREMENT_ON_EACH_STEP)
		_bitbanging_do_one_step();
#else
		_turn_unconditionally(MEASUREMENT_GRANULARITY);
#endif

		printf("[%03d]\tTOR = %d\n", i, _sharp_detects_hole());

		if (_sharp_detects_hole() != sharp_initial)
			break;
	}

	/* PAP is blocked, mechanical issue, should invalidate all actions ! */
	if (i == STEP_MAX_BEFORE_BLOCKED)
		retval = 1;

exit_point:
	_stepper_en(FALSE);

	return retval;
}

#if defined(MODULE_CALIBRATION)
void motor_pap_calib(void)
{
	motor_pap_turn_next_storage();
}
#endif

void motor_pap_unit_test(void)
{
	_stepper_en(TRUE);
	_stepper_dir_set_ccw();

	// wait 5s
	//for (uint16_t steps = 0; steps < 1000 / 4; steps ++) {
	for (uint16_t steps = 0; steps < 1000 / 6; steps ++) {
//	for (uint16_t steps = 0; steps < 1000 / (6*2); steps ++) {
		xtimer_ticks32_t current_time;

		gpio_write(GPIO_PIN(PORT_A, 11), 1);
		current_time = xtimer_now();
		//xtimer_periodic_wakeup(&current_time, (3333 / 2) * 4); // 300Hz / 4 (vitesse max)
		xtimer_periodic_wakeup(&current_time, (3333 / 2) * 6); // 300Hz
//		xtimer_periodic_wakeup(&current_time, (3333 / 2) * 6 * 2); // 300Hz

		gpio_write(GPIO_PIN(PORT_A, 11), 0);
		current_time = xtimer_now();
		//xtimer_periodic_wakeup(&current_time, (3333 / 2) * 4); // 300Hz / 4 (vitesse max)
		xtimer_periodic_wakeup(&current_time, (3333 / 2) * 6); // 300Hz
//		xtimer_periodic_wakeup(&current_time, (3333 / 2) * 6 * 2); // 300Hz
	}

	_stepper_en(FALSE);
}

