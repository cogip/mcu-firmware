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
    return !gpio_read(GPIO_STARTER);
}

int pf_is_camp_left(void)
{
    /* Color switch for coords translations */
    return !gpio_read(GPIO_CAMP);
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

void pf_front_cup_take(void)
{
    sd21_servo_reach_position(PF_SERVO_FL_CUP, PF_SERVO_STATE_CUP_TAKE);
    sd21_servo_reach_position(PF_SERVO_FC_CUP, PF_SERVO_STATE_CUP_TAKE);
    sd21_servo_reach_position(PF_SERVO_FR_CUP, PF_SERVO_STATE_CUP_TAKE);
    gpio_set(GPIO_FL_PUMP_4);
    xtimer_usleep(200 * US_PER_MS);
    gpio_set(GPIO_FC_PUMP_5);
    xtimer_usleep(200 * US_PER_MS);
    gpio_set(GPIO_FR_PUMP_6);
}

void pf_front_cup_hold(void)
{
    sd21_servo_reach_position(PF_SERVO_FL_CUP, PF_SERVO_STATE_CUP_HOLD);
    sd21_servo_reach_position(PF_SERVO_FC_CUP, PF_SERVO_STATE_CUP_HOLD);
    sd21_servo_reach_position(PF_SERVO_FR_CUP, PF_SERVO_STATE_CUP_HOLD);
}


void pf_front_cup_ramp(void)
{
 // TODO: Monter les ascenceurs face AV + et stockage des palets
/* Option 1: on baisse la fourchette  et  on stocke le rouge dans la fourchette.
    Le vert et le bleu dans la rampe
    Option 2: on bloque la rampe, on stocke les 3 palets dans la rampe
*/
    sd21_servo_reach_position(PF_SERVO_FL_ELEVATOR, PF_SERVO_STATE_ELEVATOR_TOP);
    sd21_servo_reach_position(PF_SERVO_FC_ELEVATOR, PF_SERVO_STATE_ELEVATOR_TOP);
    sd21_servo_reach_position(PF_SERVO_FR_ELEVATOR, PF_SERVO_STATE_ELEVATOR_TOP);
    xtimer_usleep(250 * US_PER_MS);

    sd21_servo_reach_position(PF_SERVO_FL_CUP, PF_SERVO_STATE_CUP_RAMP);
    sd21_servo_reach_position(PF_SERVO_FC_CUP, PF_SERVO_STATE_CUP_RAMP);
    sd21_servo_reach_position(PF_SERVO_FR_CUP, PF_SERVO_STATE_CUP_RAMP);
    xtimer_usleep(500 * US_PER_MS);

    gpio_clear(GPIO_FL_PUMP_4);
    gpio_clear(GPIO_FC_PUMP_5);
    gpio_clear(GPIO_FR_PUMP_6);

    xtimer_usleep(250 * US_PER_MS);
    pf_front_cup_hold();
    xtimer_usleep(500 * US_PER_MS);
}

void pf_back_cup_take(void)
{
    sd21_servo_reach_position(PF_SERVO_BL_CUP, PF_SERVO_STATE_CUP_TAKE);
    sd21_servo_reach_position(PF_SERVO_BC_CUP, PF_SERVO_STATE_CUP_TAKE);
    sd21_servo_reach_position(PF_SERVO_BR_CUP, PF_SERVO_STATE_CUP_TAKE);
    gpio_set(GPIO_BL_PUMP_1);
    xtimer_usleep(200 * US_PER_MS);
    gpio_set(GPIO_BC_PUMP_2);
    xtimer_usleep(200 * US_PER_MS);
    gpio_set(GPIO_BR_PUMP_3);
}

void pf_back_cup_hold(void)
{
    sd21_servo_reach_position(PF_SERVO_BL_CUP, PF_SERVO_STATE_CUP_HOLD);
    sd21_servo_reach_position(PF_SERVO_BC_CUP, PF_SERVO_STATE_CUP_HOLD);
    sd21_servo_reach_position(PF_SERVO_BR_CUP, PF_SERVO_STATE_CUP_HOLD);
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

    /* Init starter and camp selection GPIOs */
    assert(gpio_init(GPIO_CAMP, GPIO_IN) == 0);
    assert(gpio_init(GPIO_STARTER, GPIO_IN_PU) == 0);

    /* Init pump GPIOs */
    assert(gpio_init(GPIO_BL_PUMP_1, GPIO_OUT) == 0);
    assert(gpio_init(GPIO_BC_PUMP_2, GPIO_OUT) == 0);
    assert(gpio_init(GPIO_BR_PUMP_3, GPIO_OUT) == 0);
    assert(gpio_init(GPIO_FL_PUMP_4, GPIO_OUT) == 0);
    assert(gpio_init(GPIO_FC_PUMP_5, GPIO_OUT) == 0);
    assert(gpio_init(GPIO_FR_PUMP_6, GPIO_OUT) == 0);

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
