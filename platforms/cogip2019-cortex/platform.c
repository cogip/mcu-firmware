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
#include "pca9548.h"
#include "planner.h"
#include "platform.h"
#include "platform-common.h"
#include "sd21.h"

#ifdef CALIBRATION
#include "calibration/calib_sd21.h"
#include "calibration/calib_pca9548.h"
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
    pca9548_init();

    for (vl53l0x_t dev = 0; dev < VL53L0X_NUMOF; dev++) {
        pca9548_set_current_channel(PCA9548_SENSORS, vl53l0x_channel[dev]);
        if (vl53l0x_init_dev(dev) != 0)
            printf("ERROR: Sensor %u init failed !!!\n", dev);
    }


#ifdef CALIBRATION
    sd21_calib_init();
    pca9548_calib_init();
#endif /* CALIBRATION */
}
