/* System includes */
#include <stdarg.h>
#include <stdio.h>
#include <thread.h>

/* RIOT includes */
#define ENABLE_DEBUG        (1)
#include "debug.h"
#include <motor_driver.h>
#include <periph/adc.h>
#include <periph/qdec.h>
#include "xtimer.h"

/* Project includes */
#include "ctrl/quadpid.h"
#include "obstacle.h"
#include "pca9548.h"
#include "planner.h"
#include "app.h"
#include "platform.h"
#include "sd21.h"

#ifdef CALIBRATION
#include "calibration/calib_pca9548.h"
#include "calibration/calib_planner.h"
#include "calibration/calib_quadpid.h"
#include "calibration/calib_sd21.h"
#endif /* CALIBRATION */

static pf_actions_context_t pf_actions_ctx = {0};

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

void pf_calib_read_sensors(pca9548_t dev)
{
    vl53l0x_t sensor = 0;
    uint8_t channel = pca9548_get_current_channel(dev);
    for (sensor = 0; sensor < VL53L0X_NUMOF; sensor++) {
        if (vl53l0x_channel[sensor] == channel)
            break;
    }

    if (sensor < VL53L0X_NUMOF) {
        uint16_t measure = vl53l0x_continuous_ranging_get_measure(dev);

        printf("Measure sensor %u: %u\n\n", sensor, measure);
    }
    else {
        printf("No sensor for this channel %u !\n\n", sensor);
    }
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

    if(!pf_actions_ctx.any_pump_on)
    {
        for (vl53l0x_t dev = 0; dev < VL53L0X_NUMOF; dev++) {

            uint16_t measure;
            irq_disable();

            pca9548_set_current_channel(PCA9548_SENSORS, vl53l0x_channel[dev]);
            measure = vl53l0x_continuous_ranging_get_measure(dev);

            irq_enable();
            DEBUG("Measure sensor %u: %u\n\n", dev, measure);

            if ((measure > OBSTACLE_DETECTION_MINIMUM_TRESHOLD)
                    && (measure < OBSTACLE_DETECTION_MAXIMUM_TRESHOLD)) {
                const pf_sensor_t* sensor = &pf_sensors[dev];
                pose_t robot_pose = *ctrl_get_pose_current(ctrl);

                res = add_dyn_obstacle(dev, &robot_pose, sensor->angle_offset, sensor->distance_offset, (double)measure);

                if (!res) {
                    obstacle_found = 1;
                }
            }

        }
    }

    return obstacle_found;
}

static void pf_vl53l0x_reset(void)
{
    for (vl53l0x_t dev = 0; dev < VL53L0X_NUMOF; dev++) {
        pca9548_set_current_channel(PCA9548_SENSORS, vl53l0x_channel[dev]);
        if (vl53l0x_reset_dev(dev) != 0)
            DEBUG("ERROR: Sensor %u reset failed !!!\n", dev);
    }
}

static void pf_vl53l0x_init(void)
{
    for (vl53l0x_t dev = 0; dev < VL53L0X_NUMOF; dev++) {
        pca9548_set_current_channel(PCA9548_SENSORS, vl53l0x_channel[dev]);
        if (vl53l0x_init_dev(dev) != 0)
            DEBUG("ERROR: Sensor %u init failed !!!\n", dev);
    }
}

void pf_stop_pumps(void)
{
    gpio_clear(GPIO_BL_PUMP_1);
    gpio_clear(GPIO_BC_PUMP_2);
    gpio_clear(GPIO_BR_PUMP_3);
    gpio_clear(GPIO_FL_PUMP_4);
    gpio_clear(GPIO_FC_PUMP_5);
    gpio_clear(GPIO_FR_PUMP_6);
    pf_actions_ctx.any_pump_on = 0;
}


void pf_front_cup_take(void)
{
    sd21_servo_reach_position(PF_SERVO_FL_CUP, PF_SERVO_STATE_CUP_TAKE);
    sd21_servo_reach_position(PF_SERVO_FC_CUP, PF_SERVO_STATE_CUP_TAKE);
    sd21_servo_reach_position(PF_SERVO_FR_CUP, PF_SERVO_STATE_CUP_TAKE);

    pf_actions_ctx.any_pump_on = 1;

    gpio_set(GPIO_FL_PUMP_4);
    xtimer_usleep(200 * US_PER_MS);
    gpio_set(GPIO_FC_PUMP_5);
    xtimer_usleep(200 * US_PER_MS);
    gpio_set(GPIO_FR_PUMP_6);
    xtimer_usleep(10 * US_PER_MS);
}


void pf_front_cup_ramp(void)
{
 // TODO: Monter les ascenceurs face AV + et stockage des palets
/* Option 1: on baisse la fourchette  et  on stocke le rouge dans la fourchette.
    Le vert et le bleu dans la rampe
    Option 2: on bloque la rampe, on stocke les 3 palets dans la rampe
*/
    if (pf_actions_ctx.nb_puck_front_ramp != 0)
        return;
    sd21_servo_reach_position(PF_SERVO_FL_ELEVATOR, PF_SERVO_STATE_ELEVATOR_TOP);
    sd21_servo_reach_position(PF_SERVO_FC_ELEVATOR, PF_SERVO_STATE_ELEVATOR_TOP);
    sd21_servo_reach_position(PF_SERVO_FR_ELEVATOR, PF_SERVO_STATE_ELEVATOR_TOP);
    xtimer_usleep(250 * US_PER_MS);
    sd21_servo_reach_position(PF_SERVO_FC_CUP, PF_SERVO_STATE_CUP_RAMP);
    pf_actions_ctx.nb_puck_front_ramp = 3;
    sd21_servo_reach_position(PF_SERVO_FL_CUP, PF_SERVO_STATE_CUP_RAMP);
    sd21_servo_reach_position(PF_SERVO_FR_CUP, PF_SERVO_STATE_CUP_RAMP);
    xtimer_usleep(500 * US_PER_MS);

    gpio_clear(GPIO_FL_PUMP_4);
    gpio_clear(GPIO_FC_PUMP_5);
    gpio_clear(GPIO_FR_PUMP_6);

    pf_actions_ctx.any_pump_on = 0;

    xtimer_usleep(250 * US_PER_MS);
    sd21_servo_reach_position(PF_SERVO_FL_CUP, PF_SERVO_STATE_CUP_HOLD);
    sd21_servo_reach_position(PF_SERVO_FC_CUP, PF_SERVO_STATE_CUP_HOLD);
    sd21_servo_reach_position(PF_SERVO_FR_CUP, PF_SERVO_STATE_CUP_HOLD);
    xtimer_usleep(500 * US_PER_MS);

    sd21_servo_reach_position(PF_SERVO_FL_ELEVATOR, PF_SERVO_STATE_ELEVATOR_BOTTOM);
    sd21_servo_reach_position(PF_SERVO_FC_ELEVATOR, PF_SERVO_STATE_ELEVATOR_BOTTOM);
    sd21_servo_reach_position(PF_SERVO_FR_ELEVATOR, PF_SERVO_STATE_ELEVATOR_BOTTOM);

    pf_vl53l0x_reset();
    pf_vl53l0x_init();
}

void pf_back_cup_take(void)
{
    sd21_servo_reach_position(PF_SERVO_BL_CUP, PF_SERVO_STATE_CUP_TAKE);
    sd21_servo_reach_position(PF_SERVO_BC_CUP, PF_SERVO_STATE_CUP_TAKE);
    sd21_servo_reach_position(PF_SERVO_BR_CUP, PF_SERVO_STATE_CUP_TAKE);

    pf_actions_ctx.any_pump_on = 1;

    gpio_set(GPIO_BL_PUMP_1);
    xtimer_usleep(200 * US_PER_MS);
    gpio_set(GPIO_BC_PUMP_2);
    xtimer_usleep(200 * US_PER_MS);
    gpio_set(GPIO_BR_PUMP_3);
    xtimer_usleep(10 * US_PER_MS);
}

void pf_back_cup_ramp(void)
{
    sd21_servo_reach_position(PF_SERVO_BL_ELEVATOR, PF_SERVO_STATE_ELEVATOR_TOP);
    sd21_servo_reach_position(PF_SERVO_BC_ELEVATOR, PF_SERVO_STATE_ELEVATOR_TOP);
    sd21_servo_reach_position(PF_SERVO_BR_ELEVATOR, PF_SERVO_STATE_ELEVATOR_TOP);
    xtimer_usleep(250 * US_PER_MS);

    sd21_servo_reach_position(PF_SERVO_BL_CUP, PF_SERVO_STATE_CUP_RAMP);
    sd21_servo_reach_position(PF_SERVO_BC_CUP, PF_SERVO_STATE_CUP_RAMP);
    sd21_servo_reach_position(PF_SERVO_BR_CUP, PF_SERVO_STATE_CUP_RAMP);
    xtimer_usleep(500 * US_PER_MS);

    gpio_clear(GPIO_BL_PUMP_1);
    gpio_clear(GPIO_BC_PUMP_2);
    gpio_clear(GPIO_BR_PUMP_3);

    pf_actions_ctx.any_pump_on = 0;
    pf_actions_ctx.nb_puck_back_ramp = 3;

    xtimer_usleep(250 * US_PER_MS);
    sd21_servo_reach_position(PF_SERVO_BL_CUP, PF_SERVO_STATE_CUP_HOLD);
    sd21_servo_reach_position(PF_SERVO_BC_CUP, PF_SERVO_STATE_CUP_HOLD);
    sd21_servo_reach_position(PF_SERVO_BR_CUP, PF_SERVO_STATE_CUP_HOLD);
    xtimer_usleep(500 * US_PER_MS);

    sd21_servo_reach_position(PF_SERVO_BL_ELEVATOR, PF_SERVO_STATE_ELEVATOR_BOTTOM);
    sd21_servo_reach_position(PF_SERVO_BC_ELEVATOR, PF_SERVO_STATE_ELEVATOR_BOTTOM);
    sd21_servo_reach_position(PF_SERVO_BR_ELEVATOR, PF_SERVO_STATE_ELEVATOR_BOTTOM);

    pf_vl53l0x_reset();
    pf_vl53l0x_init();
}


void pf_front_ramp_right_drop(void)
{
    if(pf_actions_ctx.nb_puck_front_ramp == 0)
        return;

    uint8_t is_camp_left = pf_is_camp_left();

    sd21_servo_reach_position(PF_SERVO_F_RAMP_BLOCK, PF_SERVO_STATE_RAMP_OPEN);
    if (is_camp_left) {
        sd21_servo_reach_position(PF_SERVO_F_RAMP, PF_SERVO_STATE_RAMP_LEFT);
        sd21_servo_reach_position(PF_SERVO_FL_RAMP_DISP,
                PF_SERVO_STATE_RAMP_OPEN);
    }
    else {
        sd21_servo_reach_position(PF_SERVO_F_RAMP, PF_SERVO_STATE_RAMP_RIGHT);
        sd21_servo_reach_position(PF_SERVO_FR_RAMP_DISP,
                PF_SERVO_STATE_RAMP_OPEN);
    }
    pf_actions_ctx.nb_puck_front_ramp = 0;
    xtimer_usleep(1500 * US_PER_MS);

    pf_front_ramp_reset();
}

void pf_front_ramp_reset(void)
{
    sd21_servo_reach_position(PF_SERVO_FL_RAMP_DISP, PF_SERVO_STATE_RAMP_CLOSE);
    sd21_servo_reach_position(PF_SERVO_FR_RAMP_DISP, PF_SERVO_STATE_RAMP_CLOSE);
    xtimer_usleep(500 * US_PER_MS);
    sd21_servo_reach_position(PF_SERVO_F_RAMP, PF_SERVO_STATE_RAMP_HORIZ);
    xtimer_usleep(1000 * US_PER_MS);
}


void pf_back_ramp_left_drop(void)
{
    if(pf_actions_ctx.nb_puck_back_ramp == 0)
        return;
    uint8_t is_camp_left = pf_is_camp_left();

    sd21_servo_reach_position(PF_SERVO_B_RAMP_BLOCK, PF_SERVO_STATE_RAMP_OPEN);
    if (is_camp_left) {
        sd21_servo_reach_position(PF_SERVO_B_RAMP, PF_SERVO_STATE_RAMP_RIGHT);
        sd21_servo_reach_position(PF_SERVO_BR_RAMP_DISP, PF_SERVO_STATE_RAMP_OPEN);
        if(pf_actions_ctx.nb_puck_front_ramp == 3) {
            pf_front_ramp_right_drop();
        }
    }
    else {
        sd21_servo_reach_position(PF_SERVO_B_RAMP, PF_SERVO_STATE_RAMP_LEFT);
        sd21_servo_reach_position(PF_SERVO_BL_RAMP_DISP, PF_SERVO_STATE_RAMP_OPEN);
        if(pf_actions_ctx.nb_puck_front_ramp == 3) {
            pf_front_ramp_right_drop();
        }
    }
    pf_actions_ctx.nb_puck_back_ramp = 0;
    xtimer_usleep(1500 * US_PER_MS);

    pf_back_ramp_reset();
}

void pf_back_ramp_reset(void)
{
    sd21_servo_reach_position(PF_SERVO_BL_RAMP_DISP, PF_SERVO_STATE_RAMP_CLOSE);
    sd21_servo_reach_position(PF_SERVO_BR_RAMP_DISP, PF_SERVO_STATE_RAMP_CLOSE);
    xtimer_usleep(500 * US_PER_MS);
    sd21_servo_reach_position(PF_SERVO_B_RAMP, PF_SERVO_STATE_RAMP_HORIZ);
    xtimer_usleep(1000 * US_PER_MS);
}

void pf_back_ramp_left_horiz_for_goldenium(void)
{
    uint8_t is_camp_left = pf_is_camp_left();

    sd21_servo_reach_position(PF_SERVO_B_RAMP_BLOCK, PF_SERVO_STATE_RAMP_OPEN);
    if (is_camp_left)
        sd21_servo_reach_position(PF_SERVO_BR_RAMP_DISP, PF_SERVO_STATE_RAMP_OPEN);
    else
        sd21_servo_reach_position(PF_SERVO_BL_RAMP_DISP, PF_SERVO_STATE_RAMP_OPEN);
    xtimer_usleep(500 * US_PER_MS);

    pf_actions_ctx.goldenium_opened = 1;
}

void pf_arms_open(void)
{
    if(pf_actions_ctx.any_pump_on)
        pf_stop_pumps();

    if(pf_actions_ctx.front_arms_opened == 1)
        return;
    sd21_servo_reach_position(PF_SERVO_FL_ARM, PF_SERVO_STATE_ARM_OPEN);
    sd21_servo_reach_position(PF_SERVO_FR_ARM, PF_SERVO_STATE_ARM_OPEN);
    xtimer_usleep(500 * US_PER_MS);

    pf_actions_ctx.front_arms_opened = 1;
}

void pf_arms_close(void)
{
    if(pf_actions_ctx.any_pump_on)
        pf_stop_pumps();

    if(pf_actions_ctx.front_arms_opened == 0)
        return;
    sd21_servo_reach_position(PF_SERVO_FL_ARM, PF_SERVO_STATE_ARM_CLOSE);
    sd21_servo_reach_position(PF_SERVO_FR_ARM, PF_SERVO_STATE_ARM_CLOSE);
    xtimer_usleep(500 * US_PER_MS);

    pf_actions_ctx.front_arms_opened = 0;
}

void pf_goldenium_take(void)
{
    uint8_t is_camp_left = pf_is_camp_left();

    if(!is_camp_left)
    {
        // Front Right cup do the job
        sd21_servo_reach_position(PF_SERVO_FR_CUP, PF_SERVO_STATE_CUP_TAKE);
        sd21_servo_reach_position(PF_SERVO_FR_ELEVATOR, PF_SERVO_STATE_ELEVATOR_GOLDEN);
        pf_actions_ctx.any_pump_on = 1;
        gpio_set(GPIO_FL_PUMP_4);
        xtimer_usleep(10 * US_PER_MS);
    }
    else
    {
        // Front Left cup do the job
        sd21_servo_reach_position(PF_SERVO_FL_CUP, PF_SERVO_STATE_CUP_TAKE);
        sd21_servo_reach_position(PF_SERVO_FL_ELEVATOR, PF_SERVO_STATE_ELEVATOR_GOLDEN);
        pf_actions_ctx.any_pump_on = 1;
        gpio_set(GPIO_FR_PUMP_6);
        xtimer_usleep(10 * US_PER_MS);
    }
}

void pf_goldenium_hold(void)
{
    uint8_t is_camp_left = pf_is_camp_left();

    if(!is_camp_left)
    {
        // Front Right cup do the job
        sd21_servo_reach_position(PF_SERVO_FR_CUP, PF_SERVO_STATE_CUP_HOLD); //TODO: besoin de descendre ascenseur ?
        xtimer_usleep(500 * US_PER_MS);
        sd21_servo_reach_position(PF_SERVO_FR_ELEVATOR, PF_SERVO_STATE_ELEVATOR_TOP);
        xtimer_usleep(500 * US_PER_MS);
        gpio_clear(GPIO_FL_PUMP_4);
        pf_actions_ctx.any_pump_on = 0;
    }
    else
    {
        // Front Left cup do the job
        sd21_servo_reach_position(PF_SERVO_FL_CUP, PF_SERVO_STATE_CUP_HOLD); //TODO: besoin de descendre ascenseur ?
        xtimer_usleep(500 * US_PER_MS);
        sd21_servo_reach_position(PF_SERVO_FL_ELEVATOR, PF_SERVO_STATE_ELEVATOR_TOP);
        xtimer_usleep(500 * US_PER_MS);
        gpio_clear(GPIO_FR_PUMP_6);
        pf_actions_ctx.any_pump_on = 0;
    }
}

void pf_goldenium_drop(void)
{
    uint8_t is_camp_left = pf_is_camp_left();

    if(!is_camp_left)
    {
        // Front Right cup do the job
        pf_actions_ctx.any_pump_on = 1;
        gpio_set(GPIO_FL_PUMP_4);
        xtimer_usleep(500 * US_PER_MS);
        sd21_servo_reach_position(PF_SERVO_FR_ELEVATOR, PF_SERVO_STATE_ELEVATOR_BOTTOM);
        xtimer_usleep(500 * US_PER_MS);
        sd21_servo_reach_position(PF_SERVO_FR_CUP, PF_SERVO_STATE_CUP_TAKE);
        xtimer_usleep(500 * US_PER_MS);
        gpio_clear(GPIO_FL_PUMP_4);
        pf_actions_ctx.any_pump_on = 0;
    }
    else
    {
        // Front Left cup do the job
        pf_actions_ctx.any_pump_on = 1;
        gpio_set(GPIO_FR_PUMP_6);
        xtimer_usleep(500 * US_PER_MS);
        sd21_servo_reach_position(PF_SERVO_FL_ELEVATOR, PF_SERVO_STATE_ELEVATOR_BOTTOM);
        xtimer_usleep(500 * US_PER_MS);
        sd21_servo_reach_position(PF_SERVO_FL_CUP, PF_SERVO_STATE_CUP_TAKE);
        xtimer_usleep(500 * US_PER_MS);
        gpio_clear(GPIO_FR_PUMP_6);
        pf_actions_ctx.any_pump_on = 0;
    }
}

void pf_init(void)
{
    board_init();

    motor_driver_init(MOTOR_DRIVER_DEV(0));

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

    /* Debug LED */
    assert(gpio_init(GPIO_DEBUG_LED, GPIO_OUT) == 0);
    gpio_clear(GPIO_DEBUG_LED);

    sd21_init();
    pca9548_init();

    for (vl53l0x_t dev = 0; dev < VL53L0X_NUMOF; dev++) {
        pca9548_set_current_channel(PCA9548_SENSORS, vl53l0x_channel[dev]);
        if (vl53l0x_init_dev(dev) != 0)
            printf("ERROR: Sensor %u init failed !!!\n", dev);
    }

    pf_fixed_obstacles_init();

    ctrl_set_anti_blocking_on(pf_get_ctrl(), TRUE);

#ifdef CALIBRATION
    ctrl_quadpid_calib_init();
    pca9548_calib_init();
    pln_calib_init();
    sd21_calib_init();
#endif /* CALIBRATION */

}
