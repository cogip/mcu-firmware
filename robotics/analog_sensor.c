#include "analog_sensor.h"
//#include "console.h"
//#include "kos.h"
#include "platform.h"
#include "xtimer.h"

void *task_analog_sensors(void *arg)
{
	analog_sensors_t *as = (analog_sensors_t *)arg;
	uint8_t j, i;
	uint16_t sum = 0;
	uint16_t value = 0;

	for (;;)
	{
		/* current index acquisition */
		i = as->sensor_index;

		xtimer_ticks32_t loop_start_time = xtimer_now();

		value = adc_sample(as->sensors[i].adc, ADC_RES);

		/* 10-bits to 8-bits conversion */
		value >>= 2;
		value &= 0xff;

		/* save raw value in context */
		for (j = 0; j < ANALOG_SENSOR_NB_SAMPLES - 1; j++) {
			sum += as->sensors[i].raw_values[j];
			as->sensors[i].raw_values[j] = as->sensors[i].raw_values[j+1];
		}
		as->sensors[i].raw_values[j] = (value + sum) / ANALOG_SENSOR_NB_SAMPLES;

		/* round robin acquisition using 1 ADC channel */
		as->sensor_index += 1;
		as->sensor_index %= as->sensors_nb;

		xtimer_periodic_wakeup(&loop_start_time, THREAD_PERIOD_INTERVAL);
	}
}

void analog_sensor_setup(analog_sensors_t *as)
{
	uint16_t i, j;

	if (as->sensors_nb) {
		for (i = 0; i < as->sensors_nb; i++)
		{
			for (j = 0; j < ANALOG_SENSOR_NB_SAMPLES - 1; j++)
				as->sensors[i].raw_values[j] = 0;
			adc_init(ADC_LINE(as->sensors[i].adc));
		}
	}
}

static dist_cm_t analog_sensor_adc2cm(uint16_t adc,
				    float coeff_volts, float const_volts,
				    float const_dist, uint8_t dist_max)
{
	float voltage = adc * 3.3 / 255; /* 8-bits conversion, Vcc ADC = 3.3V */
	float d = voltage * coeff_volts - const_volts;
	float distance = 1 / d + const_dist;

	if (distance >= dist_max)
		distance = UINT8_MAX;

	return (dist_cm_t) distance;
}

/*
 * FIXME: following should be put elsewhere...
 */
dist_cm_t analog_sensor_detect_obstacle(analog_sensors_t *as,
					analog_sensor_zone_t zone)
{
	uint8_t i;
	dist_cm_t dist;
	dist_cm_t min_dist = AS_DIST_MAX;

	for (i = 0; i < as->sensors_nb; i++) {
		if (as->sensors[i].zone & zone) {
			uint8_t raw = as->sensors[i].raw_values[ANALOG_SENSOR_NB_SAMPLES-1];

			dist = analog_sensor_adc2cm(raw,
						    as->sensors[i].coeff_volts,
						    as->sensors[i].const_volts,
						    as->sensors[i].const_dist,
						    as->sensors[i].dist_cm_max);

			if (dist < min_dist)
				min_dist = dist;
		}
	}

	return min_dist;
}

#if defined(MODULE_CALIBRATION)
static void analog_sensor_dump_all(analog_sensors_t *as)
{
	uint8_t i;

	cons_printf("\n");

	/* Header line */
	cons_printf("\tid\t#adc\traw\tdist_cm\n");

	for (i = 0; i < as->sensors_nb; i++) {
		uint8_t raw = as->sensors[i].raw_values[ANALOG_SENSOR_NB_SAMPLES-1];
		dist_cm_t dist;

		cons_printf("\t%d\t%d\t%d", i, as->sensors[i].adc, raw >> 2);

		dist = analog_sensor_adc2cm(raw,
					    as->sensors[i].coeff_volts,
					    as->sensors[i].const_volts,
					    as->sensors[i].const_dist,
					    as->sensors[i].dist_cm_max);
		cons_printf("\t%3d", dist);

		cons_printf("\n");
	}
}

static void analog_sensor_calibration_usage(analog_sensors_t *obj)
{
	cons_printf("\n>>> Entering analog sensor calibration\n\n");

	cons_printf("sensors_nb = %d\n\n", obj->sensors_nb);
	analog_sensor_dump_all(obj);

//	cons_printf("\n");
//	cons_printf("\t'S' to start/stop measurements dumps\n");
	cons_printf("\n");
	cons_printf("\t'a','h' to display this help\n");
	cons_printf("\t'q' to quit\n");
	cons_printf("\n");
}

void analog_sensor_enter_calibration(analog_sensors_t *obj)
{
	int c = -1;
	uint8_t quit = 0;

	analog_sensor_calibration_usage(obj);

	while (!quit) {
//		if (! display_dbg) {
			/* display prompt */
			cons_printf("$ ");

			/* wait for command, or schedule */
			c = cons_getchar();
			cons_printf("%c\n", c);
//		} else {
//			/* poll for command if arrived, or dump values
//			 * periodically */
//			kos_set_next_schedule_delay_ms(200);
//
//			if (cons_is_data_arrived())
//				c = cons_getchar();
//			else
//				c = -1;
//
//			analog_sensor_dump_all(obj);
//		}

		switch (c) {
//		case -1:
//			kos_yield();
//			break;
//		case 'S':
//			display_dbg = !display_dbg;
//
//			cons_printf("display_dbg = %s\n", display_dbg ? "True" : "False");
//			break;
		case 'a':
		case 'h':
			analog_sensor_calibration_usage(obj);
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
#endif /* MODULE_CALIBRATION */
