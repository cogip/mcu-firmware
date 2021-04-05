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

void native_motor_driver_qdec_simulation(
    const motor_driver_t motor_driver, uint8_t motor_id,
    int32_t pwm_duty_cycle)
{
    static int16_t simu_motor_encoder[QDEC_NUMOF][SIMU_ENC_BUFSIZE] = { 0, };

    uint32_t i = 0, id = 0;
    int32_t s = 0;

    for (i = 0; i < motor_driver; i++) {
        const motor_driver_config_t motor_driver_conf =
            motor_driver_config[motor_driver];
        id += motor_driver_conf.nb_motors;
    }
    id += motor_id;

    if (id < QDEC_NUMOF) {

        for (i = 0; i < SIMU_ENC_BUFSIZE - 1; i++) {
            s += simu_motor_encoder[id][i];
            simu_motor_encoder[id][i] = simu_motor_encoder[id][i + 1];
        }

        s = (s + simu_motor_encoder[id][i] + pwm_duty_cycle) / (SIMU_ENC_BUFSIZE + 1);
        simu_motor_encoder[id][i] = s;
        qdecs_value[id] = s;
    }
    else {
        LOG_ERROR("MOTOR-DRIVER=%u"             \
                  "    MOTOR_ID = %u"                 \
                  "    no QDEC device associated",    \
                  motor_driver, motor_id);
    }
}

#endif /* QDEC_NUMOF */
