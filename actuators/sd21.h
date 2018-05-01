#ifndef SD21_H_
#define SD21_H_

#include <stdint.h>

#include "periph/i2c.h"

#define SD21_SERVO_OPEN		0
#define SD21_SERVO_CLOSE	1

typedef struct {
	i2c_t bus_id;
	i2c_speed_t twi_speed_khz;

	uint8_t servos_nb;
	struct {
		uint16_t value_init; /* pulse width in us */
		uint16_t value_open;
		uint16_t value_close;
	} servos[];
} sd21_t;

void sd21_setup(sd21_t *obj);
uint8_t sd21_version(sd21_t *obj);
double sd21_battery_voltage(sd21_t *obj);

void sd21_control_servo(sd21_t * obj, uint8_t servo_id, uint8_t position);

#if defined(MODULE_CALIBRATION)
void sd21_enter_calibration(sd21_t *obj);
#endif

#endif /* SD21_H_ */
