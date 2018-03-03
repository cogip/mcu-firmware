#include <stdio.h>

//#include "console.h"
#include "hbridge.h"
#include "platform.h"

//FIXME: stub to remove
#define gpio_get_input(...) 0
#define gpio_set_direction(...)
#define gpio_set_output(...)
#define timer_pwm_duty_cycle(a,b,c) ((void)c)
#define timer_pwm_mode_setup(...)
#define timer_pwm_enable(...)

#define CAL_MIN (-1000)
#define CAL_MAX (+1000)

/** limits speed command
 * @param value from -16535 to 16535
 * @return pwm value from 0 to max
 */
static uint8_t pwm_limitation(int16_t value, uint8_t offset, uint8_t max)
{
	int16_t	out = value >= 0 ? value : -value;

	if (out)
		out += offset;

	return out > max ? max : (uint8_t) out;
}

void hbridge_engine_update(hbridge_t *b, uint8_t engine_idx, int16_t pwm)
{
	engine_t *e = &b->engines[engine_idx];
	/* limits speed command */
	uint8_t pwm_period = pwm_limitation(pwm, e->offset, b->period);

	/* signed of pwm value is applied on direction gpio */
	if (e->direction_inverse_polarity)
		pwm *= -1;
	gpio_set_output(e->direction_pin_port, e->direction_pin_id, pwm > 0);

	/* generate PWM */
	timer_pwm_duty_cycle(b->tc, e->pwm_channel, pwm_period);
}

#if defined(CONFIG_CALIBRATION)
static void hbridge_calibration_usage(hbridge_t *obj)
{
	cons_printf("\n>>> Entering hbridge calibration\n\n");

	cons_printf("hbridge_nb = %d\n\n", obj->engine_nb);

	cons_printf("\t'n' to select next engine\n");
	cons_printf("\t'b' to select prev engine\n");
	cons_printf("\t'r' to reset current setting to 0\n");
	cons_printf("\t'+' to add 25\n");
	cons_printf("\t'-' to sub 25\n");
	cons_printf("\n");
	cons_printf("\t'h' to display this help\n");
	cons_printf("\t'q' to quit\n");
	cons_printf("\n");
}


void hbridge_enter_calibration(hbridge_t *obj)
{
	int c;
	uint8_t quit = 0;
	static uint8_t engine_id = 0;
	int16_t cur = 0;

	hbridge_calibration_usage(obj);

	controller_set_mode(&controller, CTRL_STATE_IDLE);

	while (!quit) {
		/* display prompt */
		cons_printf("[%02d].pwm = %4d $ ",
			engine_id, cur);

		/* wait for command */
		c = cons_getchar();
		cons_printf("%c\n", c);

		switch (c) {
		case 'n':
			engine_id += 1;
			engine_id %= obj->engine_nb;
			break;
		case 'b':
			if (engine_id)
				engine_id -= 1;
			else
				engine_id = obj->engine_nb - 1;
			break;
		case 'r':
			cur = 0;
			hbridge_engine_update(obj, engine_id, cur);
			break;
		case '+':
			cur = cur + 25 > CAL_MAX ? CAL_MAX : cur + 25;
			hbridge_engine_update(obj, engine_id, cur);
			break;
		case '-':
			cur = cur - 25 < CAL_MIN ? CAL_MIN : cur - 25;
			hbridge_engine_update(obj, engine_id, cur);
			break;
		case 'h':
			hbridge_calibration_usage(obj);
			break;
		case 'q':
			quit = 1;
			break;
		default:
			cons_printf("\n");
			break;
		}
	}
}
#endif /* CONFIG_CALIBRATION */

void hbridge_setup(hbridge_t *b)
{
	uint8_t i;

	for (i = 0; i < b->engine_nb; i++) {
		//FIXME: var to use below: engine_t *e = &b->engines[i];

		/* Configure PWM pin as output */
		gpio_set_direction(b->pwm_port,
				   e->pwm_channel,
				   GPIO_DIR_OUT);

		/* Configure direction pin as output */
		gpio_set_direction(e->direction_pin_port,
				   e->direction_pin_id,
				   GPIO_DIR_OUT);
	}

	/* setup frequency waveform generation (PWM) */
	timer_pwm_mode_setup(b->tc, b->period, b->prescaler);

	for (i = 0; i < b->engine_nb; i++)
		timer_pwm_enable(b->tc, b->engines[i].pwm_channel);
}
