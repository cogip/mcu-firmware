/* System includes */
#include <stdio.h>
#include <stdarg.h>
#include <thread.h>

/* RIOT includes */
#include "log.h"
#include <motor_driver.h>
#include <periph/adc.h>
#include <periph/qdec.h>
#include "xtimer.h"

/* Project includes */
#include "calibration/calib_planner.h"
#include "calibration/calib_quadpid.h"
#include "ctrl/quadpid.h"
#include "platform.h"
#include "platform-common.h"
#include "planner.h"

int pf_is_game_launched(void)
{
    /* Starter switch */
    return 1;
}

int pf_is_camp_left(void)
{
    /* Color switch for coords translations */
    return 0;
}

int pf_read_sensors(void)
{
    return 0;
}

void pf_front_cup_take(void)
{
}

void pf_front_cup_hold(void)
{
}

void pf_front_cup_ramp(void)
{
}

void pf_back_cup_take(void)
{
}

void pf_back_cup_hold(void)
{
}

void pf_back_cup_ramp(void)
{
}

void pf_front_ramp_right_drop(void)
{
}

void pf_front_ramp_reset(void)
{
}

void pf_back_ramp_left_drop(void)
{
}

void pf_back_ramp_reset(void)
{
}

void pf_back_ramp_left_horiz_for_goldenium(void)
{
}

void pf_arms_open(void)
{
}

void pf_arms_close(void)
{
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

    pf_fixed_obstacles_init();

#ifdef CALIBRATION
    ctrl_quadpid_calib_init();
    pln_calib_init();
#endif
}
