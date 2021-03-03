/**
 * Native Board board implementation
 *
 * Copyright (C) 2018 Gilles DOFFE <g.doffe@gmail.com>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 *
 * @ingroup boards_cogip2019_cortex_native
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

/* Shared memory key used to communicate with the simulator */
static int pf_shm_key = 0;

int pf_set_shm_key(int argc, char **argv)
{
    /* Check arguments */
    if (argc != 2) {
        puts("Bad number of arguments!");
        return EXIT_FAILURE;
    }

    pf_shm_key = atoi(argv[1]);

    return EXIT_SUCCESS;
}

int pf_get_shm_key(void)
{
    return pf_shm_key;
}

#ifdef QDEC_NUMOF

#define SIMU_ENC_BUFSIZE    3
extern int32_t qdecs_value[QDEC_NUMOF];

void native_motor_driver_qdec_simulation(
    const motor_driver_t motor_driver, uint8_t motor_id,
    int32_t pwm_duty_cycle)
{
    static int16_t simu_motor_encoder[QDEC_NUMOF][SIMU_ENC_BUFSIZE] = {0,};

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
            simu_motor_encoder[id][i] = simu_motor_encoder[id][i+1];
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
