#ifndef ANALOG_SENSOR_H_
#define ANALOG_SENSOR_H_

#include <periph/adc.h>
#include <stdint.h>
#include <limits.h>

#define AS_DIST_MAX	UINT8_MAX
typedef uint8_t dist_cm_t;

#define AS_DIST_LIMIT	30 /*cm*/

/* Average measurement over time */
#define ANALOG_SENSOR_NB_SAMPLES	3

typedef struct {
	uint8_t sensor_index;  /* current sensor in acquisition */
	uint8_t sensors_nb;
	struct {
		adc_t adc;

		/* for ADC value to distance (cm) conversion */
		float coeff_volts;
		float const_volts;
		float const_dist;

		uint8_t dist_cm_max;

		uint8_t dist_robot_offset_cm;
		double angle_robot_offset;

		/* acquisition context */
		uint8_t raw_values[ANALOG_SENSOR_NB_SAMPLES]; /* keep acquired distances */
	} sensors[];
} analog_sensors_t;

void analog_sensor_refresh_all(analog_sensors_t *as);
void analog_sensor_setup(analog_sensors_t *as);
dist_cm_t analog_sensor_check_obstacle(analog_sensors_t *as, uint8_t id);

#if defined(MODULE_CALIBRATION)
void analog_sensor_enter_calibration(analog_sensors_t *obj);
#endif

void *task_analog_sensors(void *arg);

#endif /* ANALOG_SENSOR_H_ */
