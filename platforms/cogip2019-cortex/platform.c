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
#include "obstacle.h"
#include "pca9548.h"
#include "planner.h"
#include "platform.h"
#include "platform-common.h"
#include "sd21.h"

#ifdef CALIBRATION
#include "calibration/calib_pca9548.h"
#include "calibration/calib_planner.h"
#include "calibration/calib_quadpid.h"
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

int pf_read_sensors(void)
{
    int obstacle_found = 0;
    int res = 0;

    ctrl_t* ctrl = (ctrl_t*)pf_get_quadpid_ctrl();

    reset_dyn_polygons();

    if (((ctrl_quadpid_t *)ctrl)->quadpid_params.regul == CTRL_REGUL_POSE_PRE_ANGL) {
        return 0;
    }

    for (vl53l0x_t dev = 0; dev < VL53L0X_NUMOF; dev++) {

        pca9548_set_current_channel(PCA9548_SENSORS, vl53l0x_channel[dev]);
        uint16_t measure = vl53l0x_single_ranging_measure(dev);

        if ((measure < OBSTACLE_DETECTION_TRESHOLD) && (dev == 4)) {
            const pf_sensor_t* sensor = &pf_sensors[dev];
            pose_t robot_pose = *ctrl_get_pose_current(ctrl);

            res = add_dyn_obstacle(dev, &robot_pose, sensor->angle_offset, sensor->distance_offset, (double)measure);

            if (!res) {
                obstacle_found = 1;
            }
        }

    }

    return obstacle_found;
}

void pf_init(void)
{
    board_init();

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

    pf_fixed_obstacles_init();

#ifdef CALIBRATION
    ctrl_quadpid_calib_init();
    pca9548_calib_init();
    pln_calib_init();
    sd21_calib_init();
#endif /* CALIBRATION */

}
