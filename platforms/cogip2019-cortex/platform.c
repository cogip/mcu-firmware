/* System includes */
#include <stdarg.h>
#include <stdio.h>
#include <thread.h>

/* RIOT includes */
#include "log.h"
#include <motor_driver.h>
#include <periph/adc.h>
#include <periph/qdec.h>
#include "xtimer.h"

/* Project includes */
#include "ctrl/quadpid.h"
#include "planner.h"
#include "platform.h"
#include "platform-common.h"
#include "sd21.h"

#ifdef CALIBRATION
#include "calibration/calib_sd21.h"
#endif /* CALIBRATION */

int pf_is_game_launched(void)
{
    /* Starter switch */
    return gpio_read(GPIO_STARTER);
}

int pf_is_camp_left(void)
{
    /* Color switch for coords translations */
    return gpio_read(GPIO_CAMP);
}

void pf_init(void)
{
    motor_driver_init(0);

    /* setup qdec */
    int error = qdec_init(QDEC_DEV(HBRIDGE_MOTOR_LEFT), QDEC_MODE, NULL, NULL);
    if (error) {
        printf("QDEC %u not initialized, error=%d !!!\n", HBRIDGE_MOTOR_LEFT, error);
    }
    error = qdec_init(QDEC_DEV(HBRIDGE_MOTOR_RIGHT), QDEC_MODE, NULL, NULL);
    if (error) {
        printf("QDEC %u not initialized, error=%d !!!\n", HBRIDGE_MOTOR_RIGHT, error);
    }

    odometry_setup(WHEELS_DISTANCE / PULSE_PER_MM);


    assert(gpio_init(GPIO_CAMP, GPIO_IN) == 0);
    assert(gpio_init(GPIO_STARTER, GPIO_IN) == 0);

    sd21_init();

    vl53l0x_init();

#ifdef CALIBRATION
    sd21_calib_init();
#endif /* CALIBRATION */
}
