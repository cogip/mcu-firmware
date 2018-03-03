#ifndef HBRIDGE_H_
#define HBRIDGE_H_

//#include <gpio.h>
//#include <hwtimer.h>
#include <stdint.h>
#define gpio_port_t void
#define hwtimer_t void
#define tc_clksel_t int

typedef struct {
	gpio_port_t *direction_pin_port;
	uint8_t direction_pin_id;
	uint8_t direction_inverse_polarity;
	uint8_t pwm_channel;

	uint8_t offset;
} engine_t;

typedef struct {
	hwtimer_t *tc;
	uint8_t period;
	tc_clksel_t prescaler;

	gpio_port_t *pwm_port;

	uint8_t engine_nb;
	engine_t engines[];
} hbridge_t;

void hbridge_engine_update(hbridge_t *b, uint8_t engine_idx, int16_t pwm);
#if defined(CONFIG_CALIBRATION)
void hbridge_enter_calibration(hbridge_t *obj);
#endif

void hbridge_setup(hbridge_t *b);

#endif /* HBRIDGE_H_ */
