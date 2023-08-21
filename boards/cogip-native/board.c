/**
 * Native Board board implementation
 *
 * Copyright (C) 2018 Gilles DOFFE <g.doffe@gmail.com>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 *
 * @ingroup boards_cogip-native
 * @{
 * @file
 * @author  Gilles DOFFE <g.doffe@gmail.com>
 * @}
 */
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>

/* RIOT includes */
#include <board.h>
#include <log.h>

#ifdef QDEC_NUMOF

#define SIMU_ENC_BUFSIZE    3
extern int32_t qdecs_value[QDEC_NUMOF];

void cogip_native_motor_driver_qdec_simulation(
    const motor_driver_t *motor_driver, uint8_t motor_id,
    int32_t pwm_duty_cycle)
{
    static int16_t simu_motor_encoder[QDEC_NUMOF][SIMU_ENC_BUFSIZE] = { 0, };

    uint32_t i = 0;
    int32_t s = 0;

    if (motor_id < QDEC_NUMOF) {

        for (i = 0; i < SIMU_ENC_BUFSIZE - 1; i++) {
            s += simu_motor_encoder[motor_id][i];
            simu_motor_encoder[motor_id][i] = simu_motor_encoder[motor_id][i + 1];
        }

        s = (s + simu_motor_encoder[motor_id][i] + pwm_duty_cycle) / (SIMU_ENC_BUFSIZE + 1);
        simu_motor_encoder[motor_id][i] = s;
        qdecs_value[motor_id] = s;
    }
    else {
        LOG_ERROR("MOTOR-DRIVER=%p"             \
                  "    MOTOR_ID = %u"                 \
                  "    no QDEC device associated",    \
                  (void *)motor_driver, motor_id);
    }
}

#endif /* QDEC_NUMOF */
