/* System includes */
#include <stdarg.h>
#include <stdio.h>
#include <thread.h>

/* RIOT includes */
#define ENABLE_DEBUG        (0)
#include "debug.h"
#include <motor_driver.h>
#include <periph/adc.h>
#include <periph/qdec.h>
#include "xtimer.h"

/* Project includes */
#include "quadpid.h"
#include "obstacle.h"
#include "pca9548.h"
#include "planner.h"
#include "app.h"
#include "app_conf.h"
#include "platform.h"
#include "sd21.h"

static app_actions_context_t app_actions_ctx = { 0 };

int app_is_camp_left(void)
{
    /* Color switch for coords translations */
    return !gpio_read(GPIO_CAMP);
}

void app_shell_read_sensors(pca9548_t dev)
{
    vl53l0x_t sensor = 0;
    uint8_t channel = pca9548_get_current_channel(dev);

    for (sensor = 0; sensor < VL53L0X_NUMOF; sensor++) {
        if (vl53l0x_channel[sensor] == channel) {
            break;
        }
    }

    if (sensor < VL53L0X_NUMOF) {
        uint16_t measure = vl53l0x_continuous_ranging_get_measure(dev);

        printf("Measure sensor %u: %u\n\n", sensor, measure);
    }
    else {
        printf("No sensor for this channel %u !\n\n", sensor);
    }
}

static void app_vl53l0x_reset(void)
{
    for (vl53l0x_t dev = 0; dev < VL53L0X_NUMOF; dev++) {
        pca9548_set_current_channel(PCA9548_SENSORS, vl53l0x_channel[dev]);
        if (vl53l0x_reset_dev(dev) != 0) {
            DEBUG("ERROR: Sensor %u reset failed !!!\n", dev);
        }
    }
}

static void app_vl53l0x_init(void)
{
    for (vl53l0x_t dev = 0; dev < VL53L0X_NUMOF; dev++) {
        pca9548_set_current_channel(PCA9548_SENSORS, vl53l0x_channel[dev]);
        if (vl53l0x_init_dev(dev) != 0) {
            DEBUG("ERROR: Sensor %u init failed !!!\n", dev);
        }
    }
}

static void app_fixed_obstacles_init(void)
{}

void app_stop_pumps(void)
{
    gpio_clear(GPIO_BL_PUMP_1);
    gpio_clear(GPIO_BC_PUMP_2);
    gpio_clear(GPIO_BR_PUMP_3);
    gpio_clear(GPIO_FL_PUMP_4);
    gpio_clear(GPIO_FC_PUMP_5);
    gpio_clear(GPIO_FR_PUMP_6);
    app_actions_ctx.any_pump_on = 0;
}


void app_front_cup_take(void)
{
    sd21_servo_reach_position(APP_SERVO_FL_CUP, APP_SERVO_STATE_CUP_TAKE);
    sd21_servo_reach_position(APP_SERVO_FC_CUP, APP_SERVO_STATE_CUP_TAKE);
    sd21_servo_reach_position(APP_SERVO_FR_CUP, APP_SERVO_STATE_CUP_TAKE);

    app_actions_ctx.any_pump_on = 1;

    gpio_set(GPIO_FL_PUMP_4);
    xtimer_usleep(200 * US_PER_MS);
    gpio_set(GPIO_FC_PUMP_5);
    xtimer_usleep(200 * US_PER_MS);
    gpio_set(GPIO_FR_PUMP_6);
    xtimer_usleep(10 * US_PER_MS);
}


void app_front_cup_ramp(void)
{
    // TODO: Monter les ascenceurs face AV + et stockage des palets
/* Option 1: on baisse la fourchette  et  on stocke le rouge dans la fourchette.
    Le vert et le bleu dans la rampe
    Option 2: on bloque la rampe, on stocke les 3 palets dans la rampe
 */
    if (app_actions_ctx.nb_puck_front_ramp != 0) {
        return;
    }
    sd21_servo_reach_position(APP_SERVO_FL_ELEVATOR, APP_SERVO_STATE_ELEVATOR_TOP);
    sd21_servo_reach_position(APP_SERVO_FC_ELEVATOR, APP_SERVO_STATE_ELEVATOR_TOP);
    sd21_servo_reach_position(APP_SERVO_FR_ELEVATOR, APP_SERVO_STATE_ELEVATOR_TOP);
    xtimer_usleep(250 * US_PER_MS);
    sd21_servo_reach_position(APP_SERVO_FC_CUP, APP_SERVO_STATE_CUP_RAMP);
    app_actions_ctx.nb_puck_front_ramp = 3;
    sd21_servo_reach_position(APP_SERVO_FL_CUP, APP_SERVO_STATE_CUP_RAMP);
    sd21_servo_reach_position(APP_SERVO_FR_CUP, APP_SERVO_STATE_CUP_RAMP);
    xtimer_usleep(500 * US_PER_MS);

    gpio_clear(GPIO_FL_PUMP_4);
    gpio_clear(GPIO_FC_PUMP_5);
    gpio_clear(GPIO_FR_PUMP_6);

    app_actions_ctx.any_pump_on = 0;

    xtimer_usleep(250 * US_PER_MS);
    sd21_servo_reach_position(APP_SERVO_FL_CUP, APP_SERVO_STATE_CUP_HOLD);
    sd21_servo_reach_position(APP_SERVO_FC_CUP, APP_SERVO_STATE_CUP_HOLD);
    sd21_servo_reach_position(APP_SERVO_FR_CUP, APP_SERVO_STATE_CUP_HOLD);
    xtimer_usleep(500 * US_PER_MS);

    sd21_servo_reach_position(APP_SERVO_FL_ELEVATOR, APP_SERVO_STATE_ELEVATOR_BOTTOM);
    sd21_servo_reach_position(APP_SERVO_FC_ELEVATOR, APP_SERVO_STATE_ELEVATOR_BOTTOM);
    sd21_servo_reach_position(APP_SERVO_FR_ELEVATOR, APP_SERVO_STATE_ELEVATOR_BOTTOM);

    app_vl53l0x_reset();
    app_vl53l0x_init();
}

void app_back_cup_take(void)
{
    sd21_servo_reach_position(APP_SERVO_BL_CUP, APP_SERVO_STATE_CUP_TAKE);
    sd21_servo_reach_position(APP_SERVO_BC_CUP, APP_SERVO_STATE_CUP_TAKE);
    sd21_servo_reach_position(APP_SERVO_BR_CUP, APP_SERVO_STATE_CUP_TAKE);

    app_actions_ctx.any_pump_on = 1;

    gpio_set(GPIO_BL_PUMP_1);
    xtimer_usleep(200 * US_PER_MS);
    gpio_set(GPIO_BC_PUMP_2);
    xtimer_usleep(200 * US_PER_MS);
    gpio_set(GPIO_BR_PUMP_3);
    xtimer_usleep(10 * US_PER_MS);
}

void app_back_cup_ramp(void)
{
    sd21_servo_reach_position(APP_SERVO_BL_ELEVATOR, APP_SERVO_STATE_ELEVATOR_TOP);
    sd21_servo_reach_position(APP_SERVO_BC_ELEVATOR, APP_SERVO_STATE_ELEVATOR_TOP);
    sd21_servo_reach_position(APP_SERVO_BR_ELEVATOR, APP_SERVO_STATE_ELEVATOR_TOP);
    xtimer_usleep(250 * US_PER_MS);

    sd21_servo_reach_position(APP_SERVO_BL_CUP, APP_SERVO_STATE_CUP_RAMP);
    sd21_servo_reach_position(APP_SERVO_BC_CUP, APP_SERVO_STATE_CUP_RAMP);
    sd21_servo_reach_position(APP_SERVO_BR_CUP, APP_SERVO_STATE_CUP_RAMP);
    xtimer_usleep(500 * US_PER_MS);

    gpio_clear(GPIO_BL_PUMP_1);
    gpio_clear(GPIO_BC_PUMP_2);
    gpio_clear(GPIO_BR_PUMP_3);

    app_actions_ctx.any_pump_on = 0;
    app_actions_ctx.nb_puck_back_ramp = 3;

    xtimer_usleep(250 * US_PER_MS);
    sd21_servo_reach_position(APP_SERVO_BL_CUP, APP_SERVO_STATE_CUP_HOLD);
    sd21_servo_reach_position(APP_SERVO_BC_CUP, APP_SERVO_STATE_CUP_HOLD);
    sd21_servo_reach_position(APP_SERVO_BR_CUP, APP_SERVO_STATE_CUP_HOLD);
    xtimer_usleep(500 * US_PER_MS);

    sd21_servo_reach_position(APP_SERVO_BL_ELEVATOR, APP_SERVO_STATE_ELEVATOR_BOTTOM);
    sd21_servo_reach_position(APP_SERVO_BC_ELEVATOR, APP_SERVO_STATE_ELEVATOR_BOTTOM);
    sd21_servo_reach_position(APP_SERVO_BR_ELEVATOR, APP_SERVO_STATE_ELEVATOR_BOTTOM);

    app_vl53l0x_reset();
    app_vl53l0x_init();
}


void app_front_ramp_right_drop(void)
{
    if (app_actions_ctx.nb_puck_front_ramp == 0) {
        return;
    }

    uint8_t is_camp_left = app_is_camp_left();

    sd21_servo_reach_position(APP_SERVO_F_RAMP_BLOCK, APP_SERVO_STATE_RAMP_OPEN);
    if (is_camp_left) {
        sd21_servo_reach_position(APP_SERVO_F_RAMP, APP_SERVO_STATE_RAMP_LEFT);
        sd21_servo_reach_position(APP_SERVO_FL_RAMP_DISP,
                                  APP_SERVO_STATE_RAMP_OPEN);
    }
    else {
        sd21_servo_reach_position(APP_SERVO_F_RAMP, APP_SERVO_STATE_RAMP_RIGHT);
        sd21_servo_reach_position(APP_SERVO_FR_RAMP_DISP,
                                  APP_SERVO_STATE_RAMP_OPEN);
    }
    app_actions_ctx.nb_puck_front_ramp = 0;
    xtimer_usleep(1500 * US_PER_MS);

    app_front_ramp_reset();
}

void app_front_ramp_reset(void)
{
    sd21_servo_reach_position(APP_SERVO_FL_RAMP_DISP, APP_SERVO_STATE_RAMP_CLOSE);
    sd21_servo_reach_position(APP_SERVO_FR_RAMP_DISP, APP_SERVO_STATE_RAMP_CLOSE);
    xtimer_usleep(500 * US_PER_MS);
    sd21_servo_reach_position(APP_SERVO_F_RAMP, APP_SERVO_STATE_RAMP_HORIZ);
    xtimer_usleep(1000 * US_PER_MS);
}


void app_back_ramp_left_drop(void)
{
    if (app_actions_ctx.nb_puck_back_ramp == 0) {
        return;
    }
    uint8_t is_camp_left = app_is_camp_left();

    sd21_servo_reach_position(APP_SERVO_B_RAMP_BLOCK, APP_SERVO_STATE_RAMP_OPEN);
    if (is_camp_left) {
        sd21_servo_reach_position(APP_SERVO_B_RAMP, APP_SERVO_STATE_RAMP_RIGHT);
        sd21_servo_reach_position(APP_SERVO_BR_RAMP_DISP, APP_SERVO_STATE_RAMP_OPEN);
        if (app_actions_ctx.nb_puck_front_ramp == 3) {
            app_front_ramp_right_drop();
        }
    }
    else {
        sd21_servo_reach_position(APP_SERVO_B_RAMP, APP_SERVO_STATE_RAMP_LEFT);
        sd21_servo_reach_position(APP_SERVO_BL_RAMP_DISP, APP_SERVO_STATE_RAMP_OPEN);
        if (app_actions_ctx.nb_puck_front_ramp == 3) {
            app_front_ramp_right_drop();
        }
    }
    app_actions_ctx.nb_puck_back_ramp = 0;
    xtimer_usleep(1500 * US_PER_MS);

    app_back_ramp_reset();
}

void app_back_ramp_reset(void)
{
    sd21_servo_reach_position(APP_SERVO_BL_RAMP_DISP, APP_SERVO_STATE_RAMP_CLOSE);
    sd21_servo_reach_position(APP_SERVO_BR_RAMP_DISP, APP_SERVO_STATE_RAMP_CLOSE);
    xtimer_usleep(500 * US_PER_MS);
    sd21_servo_reach_position(APP_SERVO_B_RAMP, APP_SERVO_STATE_RAMP_HORIZ);
    xtimer_usleep(1000 * US_PER_MS);
}

void app_back_ramp_left_horiz_for_goldenium(void)
{
    uint8_t is_camp_left = app_is_camp_left();

    sd21_servo_reach_position(APP_SERVO_B_RAMP_BLOCK, APP_SERVO_STATE_RAMP_OPEN);
    if (is_camp_left) {
        sd21_servo_reach_position(APP_SERVO_BR_RAMP_DISP, APP_SERVO_STATE_RAMP_OPEN);
    }
    else {
        sd21_servo_reach_position(APP_SERVO_BL_RAMP_DISP, APP_SERVO_STATE_RAMP_OPEN);
    }
    xtimer_usleep(500 * US_PER_MS);

    app_actions_ctx.goldenium_opened = 1;
}

void app_arms_open(void)
{
    if (app_actions_ctx.any_pump_on) {
        app_stop_pumps();
    }

    if (app_actions_ctx.front_arms_opened == 1) {
        return;
    }
    sd21_servo_reach_position(APP_SERVO_FL_ARM, APP_SERVO_STATE_ARM_OPEN);
    sd21_servo_reach_position(APP_SERVO_FR_ARM, APP_SERVO_STATE_ARM_OPEN);
    xtimer_usleep(500 * US_PER_MS);

    app_actions_ctx.front_arms_opened = 1;
}

void app_arms_close(void)
{
    if (app_actions_ctx.any_pump_on) {
        app_stop_pumps();
    }

    if (app_actions_ctx.front_arms_opened == 0) {
        return;
    }
    sd21_servo_reach_position(APP_SERVO_FL_ARM, APP_SERVO_STATE_ARM_CLOSE);
    sd21_servo_reach_position(APP_SERVO_FR_ARM, APP_SERVO_STATE_ARM_CLOSE);
    xtimer_usleep(500 * US_PER_MS);

    app_actions_ctx.front_arms_opened = 0;
}

void app_goldenium_take(void)
{
    uint8_t is_camp_left = app_is_camp_left();

    if (!is_camp_left) {
        // Front Right cup do the job
        sd21_servo_reach_position(APP_SERVO_FR_CUP, APP_SERVO_STATE_CUP_TAKE);
        sd21_servo_reach_position(APP_SERVO_FR_ELEVATOR, APP_SERVO_STATE_ELEVATOR_GOLDEN);
        app_actions_ctx.any_pump_on = 1;
        gpio_set(GPIO_FL_PUMP_4);
        xtimer_usleep(10 * US_PER_MS);
    }
    else {
        // Front Left cup do the job
        sd21_servo_reach_position(APP_SERVO_FL_CUP, APP_SERVO_STATE_CUP_TAKE);
        sd21_servo_reach_position(APP_SERVO_FL_ELEVATOR, APP_SERVO_STATE_ELEVATOR_GOLDEN);
        app_actions_ctx.any_pump_on = 1;
        gpio_set(GPIO_FR_PUMP_6);
        xtimer_usleep(10 * US_PER_MS);
    }
}

void app_goldenium_hold(void)
{
    uint8_t is_camp_left = app_is_camp_left();

    if (!is_camp_left) {
        // Front Right cup do the job
        sd21_servo_reach_position(APP_SERVO_FR_CUP, APP_SERVO_STATE_CUP_HOLD); //TODO: besoin de descendre ascenseur ?
        xtimer_usleep(500 * US_PER_MS);
        sd21_servo_reach_position(APP_SERVO_FR_ELEVATOR, APP_SERVO_STATE_ELEVATOR_TOP);
        xtimer_usleep(500 * US_PER_MS);
        gpio_clear(GPIO_FL_PUMP_4);
        app_actions_ctx.any_pump_on = 0;
    }
    else {
        // Front Left cup do the job
        sd21_servo_reach_position(APP_SERVO_FL_CUP, APP_SERVO_STATE_CUP_HOLD); //TODO: besoin de descendre ascenseur ?
        xtimer_usleep(500 * US_PER_MS);
        sd21_servo_reach_position(APP_SERVO_FL_ELEVATOR, APP_SERVO_STATE_ELEVATOR_TOP);
        xtimer_usleep(500 * US_PER_MS);
        gpio_clear(GPIO_FR_PUMP_6);
        app_actions_ctx.any_pump_on = 0;
    }
}

void app_goldenium_drop(void)
{
    uint8_t is_camp_left = app_is_camp_left();

    if (!is_camp_left) {
        // Front Right cup do the job
        app_actions_ctx.any_pump_on = 1;
        gpio_set(GPIO_FL_PUMP_4);
        xtimer_usleep(500 * US_PER_MS);
        sd21_servo_reach_position(APP_SERVO_FR_ELEVATOR, APP_SERVO_STATE_ELEVATOR_BOTTOM);
        xtimer_usleep(500 * US_PER_MS);
        sd21_servo_reach_position(APP_SERVO_FR_CUP, APP_SERVO_STATE_CUP_TAKE);
        xtimer_usleep(500 * US_PER_MS);
        gpio_clear(GPIO_FL_PUMP_4);
        app_actions_ctx.any_pump_on = 0;
    }
    else {
        // Front Left cup do the job
        app_actions_ctx.any_pump_on = 1;
        gpio_set(GPIO_FR_PUMP_6);
        xtimer_usleep(500 * US_PER_MS);
        sd21_servo_reach_position(APP_SERVO_FL_ELEVATOR, APP_SERVO_STATE_ELEVATOR_BOTTOM);
        xtimer_usleep(500 * US_PER_MS);
        sd21_servo_reach_position(APP_SERVO_FL_CUP, APP_SERVO_STATE_CUP_TAKE);
        xtimer_usleep(500 * US_PER_MS);
        gpio_clear(GPIO_FR_PUMP_6);
        app_actions_ctx.any_pump_on = 0;
    }
}

void app_init(void)
{
    /* Init quadpid controller */
    pf_init_quadpid_params(ctrl_quadpid_params);

    app_fixed_obstacles_init();

    sd21_init(sd21_config_app);

    const uint8_t camp_left = pf_is_camp_left();
    /* 2019: Camp left is purple, right is yellow */
    printf("%s camp\n", camp_left ? "LEFT" : "RIGHT");
}

void app_init_tasks(void)
{}
