#include "vacuum_pump.h"
#include "vacuum_pump_params.h"

#define ENABLE_DEBUG (0)
#include "debug.h"

#define VACUUM_PUMP_NUMOF ARRAY_SIZE(vacuum_pump_params)    /**< number of used vaccum pumps */

/**
 * @brief   COGIP vacuum_pump descriptor
 */
typedef struct {
    vacuum_pump_params_t params;    /**< parameters */
} vacuum_pump_dev_t;

/* Allocate memory for the device descriptor */
vacuum_pump_dev_t vacuum_pump_devs[VACUUM_PUMP_NUMOF];

int vacuum_pump_init(const vacuum_pump_t vacuum_pump, const vacuum_pump_params_t *params)
{
    assert(vacuum_pump < VACUUM_PUMP_NUMOF);

    vacuum_pump_dev_t *vacuum_pump_dev = &vacuum_pump_devs[vacuum_pump];

    vacuum_pump_dev->params = *params;

    int res = gpio_init(vacuum_pump_dev->params.gpio_enable, GPIO_OUT);

    if (res) {
        printf("Error: Cannot initialize vacuum pump enable GPIO\n");
        return -1;
    }

    res = gpio_init(vacuum_pump_dev->params.gpio_test, GPIO_IN);
    if (res) {
        printf("Error: Cannot initialize vacuum pump test GPIO\n");
        return -1;
    }

    return 0;
}

void vacuum_pump_start(const vacuum_pump_t vacuum_pump)
{
    assert(vacuum_pump < VACUUM_PUMP_NUMOF);

    vacuum_pump_dev_t *vacuum_pump_dev = &vacuum_pump_devs[vacuum_pump];

    gpio_set(vacuum_pump_dev->params.gpio_enable);
}

void vacuum_pump_stop(const vacuum_pump_t vacuum_pump)
{
    assert(vacuum_pump < VACUUM_PUMP_NUMOF);

    vacuum_pump_dev_t *vacuum_pump_dev = &vacuum_pump_devs[vacuum_pump];

    gpio_clear(vacuum_pump_dev->params.gpio_enable);
}

bool vacuum_pump_is_under_pressure(const vacuum_pump_t vacuum_pump)
{
    assert(vacuum_pump < VACUUM_PUMP_NUMOF);

    vacuum_pump_dev_t *vacuum_pump_dev = &vacuum_pump_devs[vacuum_pump];

    return gpio_read(vacuum_pump_dev->params.gpio_test);
}
