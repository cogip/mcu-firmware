/*
 * Copyright (C) 2022 COGIP Robotics association
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/* Firmware includes */
#include "app.hpp"
#include "lx_servo.h"
#include "platform.hpp"
#include "vacuum_pump.h"

/// RIOT includes
#include <xtimer.h>

#define ENABLE_DEBUG (0)
#include <debug.h>

namespace cogip {

namespace app {

#define LX_NUMOF            14

/// Left arm
#define ARM_LEFT_LX_BASE    11
#define ARM_LEFT_LX_HEAD    12
#define ARM_LEFT_PUMP       0

/// Right arm
#define ARM_RIGHT_LX_BASE   9
#define ARM_RIGHT_LX_HEAD   10
#define ARM_RIGHT_PUMP      2

/// Central arm
#define ARM_CENTRAL_LX_BASE 1
#define ARM_CENTRAL_LX_MID  2
#define ARM_CENTRAL_LX_HEAD 3
#define ARM_CENTRAL_LX_LIFT 5
#define ARM_CENTRAL_PUMP    1

/// Storage wheel
#define WHEEL_LX            13

/// Storage lift
#define STORAGE_LX          4

static lx_t lx[LX_NUMOF];

void app_arms_init(void) {
    for (uint8_t i = 0; i < LX_NUMOF; i++) {
        lx_init(&lx[i], pf_lx_get_stream(), i);
    }
}

static void wait_timeout(uint32_t timeout) {
    /*xtimer_ticks32_t wait_start_time = xtimer_now();
    xtimer_periodic_wakeup(&wait_start_time, timeout * US_PER_MS);*/
    xtimer_msleep(timeout);
}

///
/// LEFT ARM
///
void app_left_arm_folded(void) {
    vacuum_pump_stop(ARM_LEFT_PUMP);
    int ret = lx_servo_move_time_wait_write(&lx[ARM_LEFT_LX_BASE], 800, 1000);
    ret += lx_servo_move_time_wait_write(&lx[ARM_LEFT_LX_HEAD], 475, 0); // position 320, 1s, for folded inside
    ret += lx_servo_move_start(&lx[ARM_LEFT_LX_BASE]);
    ret += lx_servo_move_start(&lx[ARM_LEFT_LX_HEAD]);
    wait_timeout(1000);

    if (ret) {
        DEBUG("Error: app_left_arm_folded() failed !");
    }
    else {
        DEBUG("app_left_arm_folded() OK !");
    }
}

void app_left_arm_gripping(void) {
    vacuum_pump_start(ARM_LEFT_PUMP);
    int ret = lx_servo_move_time_wait_write(&lx[ARM_LEFT_LX_BASE], 300, 500);
    ret += lx_servo_move_time_wait_write(&lx[ARM_LEFT_LX_HEAD], 475, 500);
    ret += lx_servo_move_start(&lx[ARM_LEFT_LX_BASE]);
    ret += lx_servo_move_start(&lx[ARM_LEFT_LX_HEAD]);
    wait_timeout(500);

    if (ret) {
        DEBUG("Error: app_left_arm_gripping() failed !");
    }
    else {
        DEBUG("app_left_arm_gripping() OK !");
    }
}

void app_left_arm_giving(void) {
    int ret = lx_servo_move_time_wait_write(&lx[ARM_LEFT_LX_BASE], 700, 100);
    ret += lx_servo_move_start(&lx[ARM_LEFT_LX_BASE]);
    wait_timeout(500);
    ret += lx_servo_move_time_wait_write(&lx[ARM_LEFT_LX_HEAD], 80, 100);
    ret = lx_servo_move_time_wait_write(&lx[ARM_LEFT_LX_BASE], 500, 200);
    ret += lx_servo_move_start(&lx[ARM_LEFT_LX_BASE]);
    ret += lx_servo_move_start(&lx[ARM_LEFT_LX_HEAD]);
    wait_timeout(500);

    if (ret) {
        DEBUG("Error: app_left_arm_giving() failed !");
    }
    else {
        DEBUG("app_left_arm_giving() OK !");
    }
}

///
/// RIGHT ARM
///
void app_right_arm_folded(void) {
    vacuum_pump_stop(ARM_RIGHT_PUMP);
    int ret = lx_servo_move_time_wait_write(&lx[ARM_RIGHT_LX_BASE], 800, 1000);
    ret += lx_servo_move_time_wait_write(&lx[ARM_RIGHT_LX_HEAD], 500, 0); // position 320, 1s, for folded inside
    ret += lx_servo_move_start(&lx[ARM_RIGHT_LX_BASE]);
    ret += lx_servo_move_start(&lx[ARM_RIGHT_LX_HEAD]);
    wait_timeout(1500);

    if (ret) {
        DEBUG("Error: app_right_arm_folded() failed !");
    }
    else {
        DEBUG("app_right_arm_folded() OK !");
    }
}

void app_right_arm_gripping(void) {
    vacuum_pump_start(ARM_RIGHT_PUMP);
    int ret = lx_servo_move_time_wait_write(&lx[ARM_RIGHT_LX_BASE], 300, 500);
    ret += lx_servo_move_time_wait_write(&lx[ARM_RIGHT_LX_HEAD], 500, 500);
    ret += lx_servo_move_start(&lx[ARM_RIGHT_LX_BASE]);
    ret += lx_servo_move_start(&lx[ARM_RIGHT_LX_HEAD]);
    wait_timeout(1000);

    if (ret) {
        DEBUG("Error: app_right_arm_gripping() failed !");
    }
    else {
        DEBUG("app_right_arm_gripping() OK !");
    }
}

void app_right_arm_giving(void) {
    int ret = lx_servo_move_time_wait_write(&lx[ARM_RIGHT_LX_BASE], 700, 100);
    ret += lx_servo_move_start(&lx[ARM_RIGHT_LX_BASE]);
    wait_timeout(500);
    ret += lx_servo_move_time_wait_write(&lx[ARM_RIGHT_LX_HEAD], 880, 100);
    ret += lx_servo_move_time_wait_write(&lx[ARM_RIGHT_LX_BASE], 340, 200);
    ret += lx_servo_move_start(&lx[ARM_RIGHT_LX_BASE]);
    ret += lx_servo_move_start(&lx[ARM_RIGHT_LX_HEAD]);
    wait_timeout(500);

    if (ret) {
        DEBUG("Error: app_right_arm_giving() failed !");
    }
    else {
        DEBUG("app_right_arm_giving() OK !");
    }
}

///
/// CENTRAL ARM
///
void app_central_arm_folded(void)
{
    vacuum_pump_stop(ARM_CENTRAL_PUMP);
    int ret = lx_servo_move_time_wait_write(&lx[ARM_CENTRAL_LX_LIFT], 600, 500);
    ret += lx_servo_move_time_wait_write(&lx[ARM_CENTRAL_LX_BASE], 580, 1000);
    ret += lx_servo_move_time_wait_write(&lx[ARM_CENTRAL_LX_MID], 500, 1000);
    ret += lx_servo_move_time_wait_write(&lx[ARM_CENTRAL_LX_HEAD], 500, 0);
    ret += lx_servo_move_start(&lx[ARM_CENTRAL_LX_LIFT]);
    ret += lx_servo_move_start(&lx[ARM_CENTRAL_LX_BASE]);
    ret += lx_servo_move_start(&lx[ARM_CENTRAL_LX_HEAD]);
    ret += lx_servo_move_start(&lx[ARM_CENTRAL_LX_MID]);
    wait_timeout(1500);
}

void app_central_arm_gripping_prepare(void)
{
    int ret = lx_servo_move_time_wait_write(&lx[ARM_CENTRAL_LX_BASE], 250, 500);
    ret += lx_servo_move_time_wait_write(&lx[ARM_CENTRAL_LX_MID], 950, 500);
    ret += lx_servo_move_time_wait_write(&lx[ARM_CENTRAL_LX_LIFT], 350, 500);
    ret += lx_servo_move_start(&lx[ARM_CENTRAL_LX_LIFT]);
    ret += lx_servo_move_start(&lx[ARM_CENTRAL_LX_BASE]);
    ret += lx_servo_move_start(&lx[ARM_CENTRAL_LX_MID]);
    wait_timeout(500);
    ret += lx_servo_move_time_wait_write(&lx[ARM_CENTRAL_LX_HEAD], 500, 500);
    ret += lx_servo_move_start(&lx[ARM_CENTRAL_LX_HEAD]);
    wait_timeout(500);
}

void app_central_arm_gripping(void)
{
    vacuum_pump_start(ARM_CENTRAL_PUMP);
    int ret = lx_servo_move_time_wait_write(&lx[ARM_CENTRAL_LX_LIFT], 380, 500);
    ret += lx_servo_move_start(&lx[ARM_CENTRAL_LX_LIFT]);
    wait_timeout(1000);
}

void app_central_arm_gripping_statuette(void)
{
    vacuum_pump_start(ARM_CENTRAL_PUMP);
    int ret = lx_servo_move_time_wait_write(&lx[ARM_CENTRAL_LX_BASE], 200, 500);
    ret += lx_servo_move_time_wait_write(&lx[ARM_CENTRAL_LX_MID], 500, 500);
    ret += lx_servo_move_time_wait_write(&lx[ARM_CENTRAL_LX_LIFT], 500, 500);
    ret += lx_servo_move_time_wait_write(&lx[ARM_CENTRAL_LX_HEAD], 500, 0);
    ret += lx_servo_move_start(&lx[ARM_CENTRAL_LX_LIFT]);
    ret += lx_servo_move_start(&lx[ARM_CENTRAL_LX_BASE]);
    ret += lx_servo_move_start(&lx[ARM_CENTRAL_LX_MID]);
    ret += lx_servo_move_start(&lx[ARM_CENTRAL_LX_HEAD]);
    wait_timeout(500);
}

void app_central_arm_gripping_statuette_up(void)
{
    int ret = lx_servo_move_time_wait_write(&lx[ARM_CENTRAL_LX_LIFT], 600, 500);
    ret += lx_servo_move_time_wait_write(&lx[ARM_CENTRAL_LX_BASE], 450, 500);
    ret += lx_servo_move_time_wait_write(&lx[ARM_CENTRAL_LX_MID], 500, 500);
    ret += lx_servo_move_time_wait_write(&lx[ARM_CENTRAL_LX_HEAD], 500, 0);
    ret += lx_servo_move_start(&lx[ARM_CENTRAL_LX_LIFT]);
    ret += lx_servo_move_start(&lx[ARM_CENTRAL_LX_BASE]);
    ret += lx_servo_move_start(&lx[ARM_CENTRAL_LX_HEAD]);
    ret += lx_servo_move_start(&lx[ARM_CENTRAL_LX_MID]);
    wait_timeout(500);
}

void app_central_arm_releasing_statuette(void)
{
    int ret = lx_servo_move_time_wait_write(&lx[ARM_CENTRAL_LX_BASE], 250, 500);
    ret += lx_servo_move_time_wait_write(&lx[ARM_CENTRAL_LX_MID], 500, 500);
    ret += lx_servo_move_time_wait_write(&lx[ARM_CENTRAL_LX_LIFT], 350, 500);
    ret += lx_servo_move_time_wait_write(&lx[ARM_CENTRAL_LX_HEAD], 500, 0);
    ret += lx_servo_move_start(&lx[ARM_CENTRAL_LX_LIFT]);
    ret += lx_servo_move_start(&lx[ARM_CENTRAL_LX_BASE]);
    ret += lx_servo_move_start(&lx[ARM_CENTRAL_LX_MID]);
    ret += lx_servo_move_start(&lx[ARM_CENTRAL_LX_HEAD]);
    wait_timeout(500);
    vacuum_pump_stop(ARM_CENTRAL_PUMP);
    wait_timeout(200);
    app_central_arm_folded();
}

void app_central_arm_gripping_replica(void)
{
    vacuum_pump_start(ARM_CENTRAL_PUMP);
    int ret = lx_servo_move_time_wait_write(&lx[ARM_CENTRAL_LX_BASE], 250, 500);
    ret += lx_servo_move_time_wait_write(&lx[ARM_CENTRAL_LX_MID], 500, 500);
    ret += lx_servo_move_time_wait_write(&lx[ARM_CENTRAL_LX_LIFT], 450, 500);
    ret += lx_servo_move_time_wait_write(&lx[ARM_CENTRAL_LX_HEAD], 500, 0);
    ret += lx_servo_move_start(&lx[ARM_CENTRAL_LX_LIFT]);
    ret += lx_servo_move_start(&lx[ARM_CENTRAL_LX_BASE]);
    ret += lx_servo_move_start(&lx[ARM_CENTRAL_LX_MID]);
    ret += lx_servo_move_start(&lx[ARM_CENTRAL_LX_HEAD]);
    wait_timeout(500);
}

void app_central_arm_gripping_replica_up(void)
{
    int ret = lx_servo_move_time_wait_write(&lx[ARM_CENTRAL_LX_LIFT], 600, 500);
    ret += lx_servo_move_time_wait_write(&lx[ARM_CENTRAL_LX_BASE], 450, 500);
    ret += lx_servo_move_time_wait_write(&lx[ARM_CENTRAL_LX_MID], 500, 500);
    ret += lx_servo_move_time_wait_write(&lx[ARM_CENTRAL_LX_HEAD], 500, 0);
    ret += lx_servo_move_start(&lx[ARM_CENTRAL_LX_LIFT]);
    ret += lx_servo_move_start(&lx[ARM_CENTRAL_LX_BASE]);
    ret += lx_servo_move_start(&lx[ARM_CENTRAL_LX_HEAD]);
    ret += lx_servo_move_start(&lx[ARM_CENTRAL_LX_MID]);
    wait_timeout(500);
}

void app_central_arm_releasing_replica(void)
{
    app_central_arm_releasing_statuette();
}

void app_central_arm_taking_left(void)
{
    int ret = lx_servo_move_time_wait_write(&lx[ARM_CENTRAL_LX_LIFT], 240, 500);
    ret += lx_servo_move_time_wait_write(&lx[ARM_CENTRAL_LX_BASE], 500, 500);
    ret += lx_servo_move_time_wait_write(&lx[ARM_CENTRAL_LX_MID], 50, 500);
    ret += lx_servo_move_start(&lx[ARM_CENTRAL_LX_LIFT]);
    ret += lx_servo_move_start(&lx[ARM_CENTRAL_LX_BASE]);
    ret += lx_servo_move_start(&lx[ARM_CENTRAL_LX_MID]);
    wait_timeout(1000);
    ret += lx_servo_move_time_wait_write(&lx[ARM_CENTRAL_LX_HEAD], 900, 500);
    ret += lx_servo_move_start(&lx[ARM_CENTRAL_LX_HEAD]);
    wait_timeout(1000);
    vacuum_pump_start(ARM_CENTRAL_PUMP);
}

void app_central_arm_taking_right(void)
{
    int ret = lx_servo_move_time_wait_write(&lx[ARM_CENTRAL_LX_LIFT], 340, 500);
    ret += lx_servo_move_time_wait_write(&lx[ARM_CENTRAL_LX_BASE], 500, 500);
    ret += lx_servo_move_time_wait_write(&lx[ARM_CENTRAL_LX_MID], 50, 500);
    ret += lx_servo_move_start(&lx[ARM_CENTRAL_LX_LIFT]);
    ret += lx_servo_move_start(&lx[ARM_CENTRAL_LX_BASE]);
    ret += lx_servo_move_start(&lx[ARM_CENTRAL_LX_MID]);
    wait_timeout(1000);
    ret += lx_servo_move_time_wait_write(&lx[ARM_CENTRAL_LX_HEAD], 100, 500);
    ret += lx_servo_move_start(&lx[ARM_CENTRAL_LX_HEAD]);
    wait_timeout(1000);
    vacuum_pump_start(ARM_CENTRAL_PUMP);
}

void app_central_arm_giving_wheel(void)
{
    int ret = lx_servo_move_time_wait_write(&lx[ARM_CENTRAL_LX_BASE], 350, 500);
    ret += lx_servo_move_time_wait_write(&lx[ARM_CENTRAL_LX_MID], 950, 500);
    ret += lx_servo_move_time_wait_write(&lx[ARM_CENTRAL_LX_LIFT], 350, 500);
    ret += lx_servo_move_start(&lx[ARM_CENTRAL_LX_LIFT]);
    ret += lx_servo_move_start(&lx[ARM_CENTRAL_LX_BASE]);
    ret += lx_servo_move_start(&lx[ARM_CENTRAL_LX_MID]);
    wait_timeout(500);
    ret += lx_servo_move_time_wait_write(&lx[ARM_CENTRAL_LX_HEAD], 500, 500);
    ret += lx_servo_move_start(&lx[ARM_CENTRAL_LX_HEAD]);
    wait_timeout(500);
    ret += lx_servo_move_time_wait_write(&lx[ARM_CENTRAL_LX_LIFT], 480, 300);
    ret += lx_servo_move_time_wait_write(&lx[ARM_CENTRAL_LX_BASE], 135, 500);
    ret += lx_servo_move_time_wait_write(&lx[ARM_CENTRAL_LX_MID], 142, 500);
    ret += lx_servo_move_time_wait_write(&lx[ARM_CENTRAL_LX_HEAD], 500, 250);
    ret += lx_servo_move_start(&lx[ARM_CENTRAL_LX_LIFT]);
    ret += lx_servo_move_start(&lx[ARM_CENTRAL_LX_BASE]);
    ret += lx_servo_move_start(&lx[ARM_CENTRAL_LX_HEAD]);
    ret += lx_servo_move_start(&lx[ARM_CENTRAL_LX_MID]);
    wait_timeout(500);
    ret += lx_servo_move_time_wait_write(&lx[ARM_CENTRAL_LX_LIFT], 600, 300);
    ret += lx_servo_move_time_wait_write(&lx[ARM_CENTRAL_LX_BASE], 520, 500);
    ret += lx_servo_move_time_wait_write(&lx[ARM_CENTRAL_LX_MID], 400, 500);
    ret += lx_servo_move_time_wait_write(&lx[ARM_CENTRAL_LX_HEAD], 500, 250);
    ret += lx_servo_move_start(&lx[ARM_CENTRAL_LX_LIFT]);
    ret += lx_servo_move_start(&lx[ARM_CENTRAL_LX_BASE]);
    ret += lx_servo_move_start(&lx[ARM_CENTRAL_LX_HEAD]);
    ret += lx_servo_move_start(&lx[ARM_CENTRAL_LX_MID]);
    wait_timeout(500);
    vacuum_pump_stop(ARM_CENTRAL_PUMP);
    wait_timeout(500);
    ret += lx_servo_move_time_wait_write(&lx[ARM_CENTRAL_LX_BASE], 320, 500);
    ret += lx_servo_move_time_wait_write(&lx[ARM_CENTRAL_LX_MID], 200, 300);
    ret += lx_servo_move_start(&lx[ARM_CENTRAL_LX_BASE]);
    ret += lx_servo_move_start(&lx[ARM_CENTRAL_LX_MID]);
    wait_timeout(500);
    ret += lx_servo_move_time_wait_write(&lx[WHEEL_LX], 870, 1000);
    ret += lx_servo_move_start(&lx[WHEEL_LX]);
    wait_timeout(1000);
    ret += lx_servo_move_time_wait_write(&lx[STORAGE_LX], 440, 200);
    ret += lx_servo_move_start(&lx[STORAGE_LX]);
    wait_timeout(200);
}

} // namespace app

} // namespace cogip
