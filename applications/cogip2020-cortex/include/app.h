#pragma once

#define SD21_SERVO_NUMOF        12
#define SD21_SERVO_POS_NUMOF    3
#define SD21_SERVO_POS_MIN      500
#define SD21_SERVO_POS_MAX      2500

/* Project includes */
#include "board.h"
#include "platform.h"
#include "sd21.h"
#include "vl53l0x.h"

/*
 * Machine parameters
 */


#define USART_CONSOLE   USARTC0

#define PCA9548_CALIB_CB(x) pf_calib_read_sensors(x)

static const sd21_conf_t sd21_config[] = {
    {   /* SD12 ID0 */
        .i2c_dev_id = 0,
        .i2c_address = (0xC2 >> 1),
        .i2c_speed_khz = I2C_SPEED_NORMAL,

        .servos_nb = 0,
    }
};

void app_calib_read_sensors(pca9548_t dev);

void app_init(void);
void app_init_tasks(void);
