/*
 * Copyright (C) 2022 COGIP Robotics association
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

// RIOT includes
#include <xtimer.h>

// Firmware includes
#include "pf_servos.hpp"
#include "pf_pumps.hpp"

namespace cogip {
namespace app {
namespace arms {

using ServoEnum = pf::actuators::servos::Enum;
using PumpEnum = pf::actuators::pumps::Enum;

static void _wait_timeout(uint32_t timeout) {
    xtimer_msleep(timeout);
}

// LEFT ARM
void left_arm_folded() {
    pf::actuators::pumps::get(PumpEnum::ARM_LEFT_PUMP).deactivate();
    pf::actuators::servos::parallel_move(
        {
            { ServoEnum::ARM_LEFT_BASE, 800, 1000 },
            { ServoEnum::ARM_LEFT_HEAD, 475 }
        },
        1000
    );
}

void left_arm_gripping() {
    pf::actuators::pumps::get(PumpEnum::ARM_LEFT_PUMP).activate();
    pf::actuators::servos::parallel_move(
        {
            { ServoEnum::ARM_LEFT_BASE, 300, 500 },
            { ServoEnum::ARM_LEFT_HEAD, 475, 500 }
        },
        500
    );
}

void left_arm_giving() {
    pf::actuators::servos::move({ ServoEnum::ARM_LEFT_BASE, 700, 100 }, 500 );
    pf::actuators::servos::parallel_move(
        {
            { ServoEnum::ARM_LEFT_BASE, 80, 100 },
            { ServoEnum::ARM_LEFT_HEAD, 500, 200 }
        },
        500
    );
}

// RIGHT ARM
void right_arm_folded() {
    pf::actuators::pumps::get(PumpEnum::ARM_RIGHT_PUMP).deactivate();
    pf::actuators::servos::parallel_move(
        {
            { ServoEnum::ARM_RIGHT_BASE, 800, 1000 },
            { ServoEnum::ARM_RIGHT_HEAD, 500 }
        },
        500
    );
}

void right_arm_gripping() {
    pf::actuators::pumps::get(PumpEnum::ARM_RIGHT_PUMP).activate();
    pf::actuators::servos::parallel_move(
        {
            { ServoEnum::ARM_RIGHT_BASE, 300, 500 },
            { ServoEnum::ARM_RIGHT_HEAD, 500, 500 }
        },
        1000
    );
}

void right_arm_giving() {
    pf::actuators::servos::move({ ServoEnum::ARM_RIGHT_BASE, 700, 100 }, 500 );
    pf::actuators::servos::parallel_move(
        {
            { ServoEnum::ARM_RIGHT_BASE, 340, 200 },
            { ServoEnum::ARM_RIGHT_HEAD, 880, 100 }
        },
        500
    );
}

// CENTRAL ARM
void central_arm_folded()
{
    pf::actuators::pumps::get(PumpEnum::ARM_CENTRAL_PUMP).deactivate();
    pf::actuators::servos::parallel_move(
        {
            { ServoEnum::ARM_CENTRAL_LIFT, 600, 500 },
            { ServoEnum::ARM_CENTRAL_BASE, 580, 1000 },
            { ServoEnum::ARM_CENTRAL_MID, 500, 1000 },
            { ServoEnum::ARM_CENTRAL_HEAD, 500 }
        },
        1500
    );
}

void central_arm_gripping_prepare()
{
    pf::actuators::servos::parallel_move(
        {
            { ServoEnum::ARM_CENTRAL_BASE, 250, 500 },
            { ServoEnum::ARM_CENTRAL_MID, 950, 500 },
            { ServoEnum::ARM_CENTRAL_LIFT, 350, 500 }
        },
        500
    );

    pf::actuators::servos::move({ ServoEnum::ARM_CENTRAL_HEAD, 500, 500 }, 500 );
}

void central_arm_gripping()
{
    pf::actuators::pumps::get(PumpEnum::ARM_CENTRAL_PUMP).activate();
    pf::actuators::servos::move({ ServoEnum::ARM_CENTRAL_LIFT, 380, 500 }, 1000 );
}

void central_arm_gripping_statuette()
{
    pf::actuators::pumps::get(PumpEnum::ARM_CENTRAL_PUMP).activate();
    pf::actuators::servos::parallel_move(
        {
            { ServoEnum::ARM_CENTRAL_BASE, 200, 500 },
            { ServoEnum::ARM_CENTRAL_MID, 500, 500 },
            { ServoEnum::ARM_CENTRAL_LIFT, 500, 500 },
            { ServoEnum::ARM_CENTRAL_HEAD, 500 }
        },
        500
    );
}

void central_arm_gripping_statuette_up()
{
    pf::actuators::servos::parallel_move(
        {
            { ServoEnum::ARM_CENTRAL_BASE, 450, 500 },
            { ServoEnum::ARM_CENTRAL_MID, 500, 500 },
            { ServoEnum::ARM_CENTRAL_LIFT, 600, 500 },
            { ServoEnum::ARM_CENTRAL_HEAD, 500 }
        },
        500
    );
}

void central_arm_releasing_statuette()
{
    pf::actuators::servos::parallel_move(
        {
            { ServoEnum::ARM_CENTRAL_BASE, 200, 500 },
            { ServoEnum::ARM_CENTRAL_MID, 500, 500 },
            { ServoEnum::ARM_CENTRAL_LIFT, 350, 500 },
            { ServoEnum::ARM_CENTRAL_HEAD, 500 }
        },
        500
    );
    pf::actuators::pumps::get(PumpEnum::ARM_CENTRAL_PUMP).deactivate();
    _wait_timeout(500);
    central_arm_folded();
}

void central_arm_gripping_replica()
{
    pf::actuators::pumps::get(PumpEnum::ARM_CENTRAL_PUMP).activate();
    pf::actuators::servos::parallel_move(
        {
            { ServoEnum::ARM_CENTRAL_BASE, 200, 500 },
            { ServoEnum::ARM_CENTRAL_MID, 500, 500 },
            { ServoEnum::ARM_CENTRAL_LIFT, 450, 500 },
            { ServoEnum::ARM_CENTRAL_HEAD, 500 }
        },
        500
    );
}

void central_arm_gripping_replica_up()
{
    pf::actuators::servos::parallel_move(
        {
            { ServoEnum::ARM_CENTRAL_BASE, 450, 500 },
            { ServoEnum::ARM_CENTRAL_MID, 500, 500 },
            { ServoEnum::ARM_CENTRAL_LIFT, 600, 500 },
            { ServoEnum::ARM_CENTRAL_HEAD, 500 }
        },
        500
    );
}

void central_arm_releasing_replica()
{
    central_arm_releasing_statuette();
}

void central_arm_taking_left()
{
    pf::actuators::servos::parallel_move(
        {
            { ServoEnum::ARM_CENTRAL_BASE, 500, 500 },
            { ServoEnum::ARM_CENTRAL_MID, 50, 500 },
            { ServoEnum::ARM_CENTRAL_LIFT, 240, 500 }
        },
        1000
    );
    pf::actuators::servos::move({ ServoEnum::ARM_CENTRAL_HEAD, 900, 500 }, 1000 );
    pf::actuators::pumps::get(PumpEnum::ARM_CENTRAL_PUMP).activate();
}

void central_arm_taking_right()
{
    pf::actuators::servos::parallel_move(
        {
            { ServoEnum::ARM_CENTRAL_BASE, 500, 500 },
            { ServoEnum::ARM_CENTRAL_MID, 50, 500 },
            { ServoEnum::ARM_CENTRAL_LIFT, 340, 500 }
        },
        1000
    );
    pf::actuators::servos::move({ ServoEnum::ARM_CENTRAL_HEAD, 100, 500 }, 1000 );
    pf::actuators::pumps::get(PumpEnum::ARM_CENTRAL_PUMP).activate();
}

void central_arm_giving_wheel()
{
    pf::actuators::servos::parallel_move(
        {
            { ServoEnum::ARM_CENTRAL_BASE, 350, 500 },
            { ServoEnum::ARM_CENTRAL_MID, 950, 500 },
            { ServoEnum::ARM_CENTRAL_LIFT, 350, 500 }
        },
        500
    );

    pf::actuators::servos::move({ ServoEnum::ARM_CENTRAL_HEAD, 500, 500 }, 500 );

    pf::actuators::servos::parallel_move(
        {
            { ServoEnum::ARM_CENTRAL_BASE, 135, 500 },
            { ServoEnum::ARM_CENTRAL_MID, 142, 500 },
            { ServoEnum::ARM_CENTRAL_LIFT, 480, 300 },
            { ServoEnum::ARM_CENTRAL_HEAD, 500, 250 }
        },
        500
    );

    pf::actuators::servos::parallel_move(
        {
            { ServoEnum::ARM_CENTRAL_BASE, 520, 500 },
            { ServoEnum::ARM_CENTRAL_MID, 400, 500 },
            { ServoEnum::ARM_CENTRAL_LIFT, 600, 300 },
            { ServoEnum::ARM_CENTRAL_HEAD, 500, 250 }
        },
        500
    );

    pf::actuators::pumps::get(PumpEnum::ARM_CENTRAL_PUMP).deactivate();
    _wait_timeout(500);

    pf::actuators::servos::parallel_move(
        {
            { ServoEnum::ARM_CENTRAL_BASE, 320, 500 },
            { ServoEnum::ARM_CENTRAL_MID, 200, 300 }
        },
        500
    );

    pf::actuators::servos::move({ ServoEnum::WHEEL, 870, 1000 }, 1000 );
    pf::actuators::servos::move({ ServoEnum::STORAGE, 440, 200 }, 200 );
}

void central_drop_gallery_low_prepare() {
    pf::actuators::pumps::get(PumpEnum::ARM_CENTRAL_PUMP).activate();
    pf::actuators::servos::parallel_move(
        {
            { ServoEnum::ARM_CENTRAL_BASE, 200, 500 },
            { ServoEnum::ARM_CENTRAL_MID, 500, 500 },
            { ServoEnum::ARM_CENTRAL_LIFT, 343, 500 },
            { ServoEnum::ARM_CENTRAL_HEAD, 500 }
        },
        500
    );
}

void central_drop_gallery_low_release()
{
    pf::actuators::servos::parallel_move(
        {
            { ServoEnum::ARM_CENTRAL_BASE, 200, 500 },
            { ServoEnum::ARM_CENTRAL_MID, 500, 500 },
            { ServoEnum::ARM_CENTRAL_LIFT, 343, 500 },
            { ServoEnum::ARM_CENTRAL_HEAD, 500 }
        },
        500
    );
    pf::actuators::pumps::get(PumpEnum::ARM_CENTRAL_PUMP).deactivate();
    _wait_timeout(1000);
    central_arm_folded();
}

void central_drop_gallery_high_prepare() {
    pf::actuators::pumps::get(PumpEnum::ARM_CENTRAL_PUMP).activate();
    pf::actuators::servos::parallel_move(
        {
            { ServoEnum::ARM_CENTRAL_BASE, 576, 500 },
            { ServoEnum::ARM_CENTRAL_MID, 793, 500 },
            { ServoEnum::ARM_CENTRAL_LIFT, 343, 500 },
            { ServoEnum::ARM_CENTRAL_HEAD, 500 }
        },
        500
    );
}

void central_drop_gallery_high_release()
{
    pf::actuators::servos::parallel_move(
        {
            { ServoEnum::ARM_CENTRAL_BASE, 576, 500 },
            { ServoEnum::ARM_CENTRAL_MID, 793, 500 },
            { ServoEnum::ARM_CENTRAL_LIFT, 343, 500 },
            { ServoEnum::ARM_CENTRAL_HEAD, 500 }
        },
        500
    );
    pf::actuators::pumps::get(PumpEnum::ARM_CENTRAL_PUMP).deactivate();
    _wait_timeout(1000);
    central_arm_folded();
}

} // namespace arms
} // namespace app
} // namespace cogip
