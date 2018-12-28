#include "analog_sensor.h"
#include "platform.h"
#include "xtimer.h"

void *task_analog_sensors(void *arg)
{
    analog_sensors_t *as = (analog_sensors_t *)arg;

    for (;;) {
        uint8_t j, i;
#if defined(AVERAGING)
        uint32_t sum = 0;
#endif
        uint16_t value = 0;

        /* current index acquisition */
        i = as->sensor_index;

        xtimer_ticks32_t loop_start_time = xtimer_now();

        value = adc_sample(as->sensors[i].adc, ADC_RES);

        /* TODO: Understand why *2 */
        //value <<= 1;

#if defined(AVERAGING)
        /* save raw value in context */
        for (j = 0; j < ANALOG_SENSOR_NB_SAMPLES - 1; j++) {
            sum += as->sensors[i].raw_values[j];
            as->sensors[i].raw_values[j] = as->sensors[i].raw_values[j + 1];
        }
        as->sensors[i].raw_values[j] = (value + sum) / ANALOG_SENSOR_NB_SAMPLES;
#else
        for (j = 0; j < ANALOG_SENSOR_NB_SAMPLES; j++) {
            as->sensors[i].raw_values[j] = value;
        }
#endif

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
        for (i = 0; i < as->sensors_nb; i++) {
            for (j = 0; j < ANALOG_SENSOR_NB_SAMPLES - 1; j++) {
                as->sensors[i].raw_values[j] = 0;
            }
            adc_init(ADC_LINE(as->sensors[i].adc));
        }
    }
}

static dist_cm_t analog_sensor_adc2cm(uint16_t adc,
                                      float coeff_volts, float const_volts,
                                      float const_dist, uint8_t dist_max)
{
    float voltage = (((float)adc) * 3.3) / 255.0; /* 8-bits conversion, Vcc ADC = 3.3V */
    float d = voltage * coeff_volts - const_volts;
    float distance = 1 / d + const_dist;

    distance /= 2.0;

    if ((distance >= dist_max) || (distance < 0)) {
        distance = AS_DIST_MAX;
    }

    return (dist_cm_t) distance;
}

dist_cm_t analog_sensor_check_obstacle(analog_sensors_t *as,
                                       uint8_t id)
{
    dist_cm_t dist = AS_DIST_MAX;

    if (id < as->sensors_nb) {
        uint8_t raw;

        raw = as->sensors[id].raw_values[ANALOG_SENSOR_NB_SAMPLES - 1];
        dist = analog_sensor_adc2cm(raw,
                                    as->sensors[id].coeff_volts,
                                    as->sensors[id].const_volts,
                                    as->sensors[id].const_dist,
                                    as->sensors[id].dist_cm_max);
    }

    return dist;
}
