/*
 * Copyright (C) 2019 COGIP Robotics association <cogip35@gmail.com>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    sd21 Servomotors SD21 board driver
 * @ingroup     drivers
 * @brief       Servomotors SD21 board driver
 *
 * Servomotors SD21 board drives analogic servomotors. It offers servomotors
 * control trough an I2C bus.
 *
 * Each servomotor can be configured with 3 bytes:
 * * byte 0: speed
 * * byte 1: position LSB in milliseconds
 * * byte 2: position MSB in milliseconds
 *
 * The driver setup the I2C address to request according to servomotor id and
 * add the byte offset to configure.
 * * servomotor_id * 3
 *
 * Two functions can be used to drive a given servomotor:
 * * sd21_servo_control:        drive the servo to a given poition at a given
 *                              speed
 * * sd21_servo_reach_position: drive the servo to a pre defined position at
 *                              default servomotor speed
 *
 * Two SD21 specific register can be accessed to get SD21 firmware version and
 * battery voltage. Two functions are provided for that purpose:
 * * sd21_get_version
 * * sd21_get_battery_voltage 
 *
 * Position has to be setup in one I2C request of 2 bytes.
 *
 * @{
 * @file
 * @brief       Common controllers API and datas
 *
 * @author      Gilles DOFFE <g.doffe@gmail.com>
 * @author      Yannick GICQUEL <yannick.gicquel@gmail.com>
 * @author      Stephen CLYMANS <sclymans@gmail.com>
 */

#pragma once

/* RIOT includes */
#include "periph/i2c.h"

/**
 * Note:    All following macros can be overrided by setting CFLAGS variable in
 *          Makefile. Example
 *              CFLAGS += -DSD21_SERVO_POS_NUMOF=3
 */

/**
 * @brief   Maximum number of servos by board
 */
#ifndef SD21_I2C_RETRIES
#define SD21_I2C_RETRIES        3
#endif /* SD21_I2C_RETRIES */

/**
 * @brief   Maximum number of servos by board
 */
#ifndef SD21_SERVO_NUMOF
#define SD21_SERVO_NUMOF        21
#endif /* SD21_SERVO_NUMOF */

/**
 * @brief   Number of predefined positions, at least 2, opened and closed
 */
#ifndef SD21_SERVO_POS_NUMOF
#define SD21_SERVO_POS_NUMOF    2
#endif /* SD21_SERVO_POS_NUMOF */

/**
 * @brief   Minimal position (in microseconds)
 */
#ifndef SD21_SERVO_POS_MIN
#define SD21_SERVO_POS_MIN    1000
#endif /* SD21_SERVO_POS_MIN */

/**
 * @brief   Maximal position (in microseconds)
 */
#ifndef SD21_SERVO_POS_MAX
#define SD21_SERVO_POS_MAX    2000
#endif /* SD21_SERVO_POS_MAX */

/**
 * @brief   Centered position (in microseconds)
 */
#define SD21_SERVO_POS_MID    ((SD21_SERVO_POS_MAX + SD21_SERVO_POS_MIN) / 2)

/**
 * @brief   Name string maximum length
 */
#define SD21_SERVO_NAME_LENGTH  64

/**
 * @brief   SD21 I2C board id
 */
typedef unsigned int sd21_t;

/**
 * @brief   Servotmotors pre-defined positions
 */
typedef enum {
    SD21_SERVO_POS_OPEN,    /**< Servomotor opened position */
    SD21_SERVO_POS_CLOSE,   /**< Servomotor closed position */
} sd21_servo_pos_t;

/**
 * @brief   Servomotor properties
 */
typedef struct {
    uint16_t positions[SD21_SERVO_POS_NUMOF];   /**< Predifined positions */
    sd21_servo_pos_t default_position;          /**< Default reset position */
    uint8_t default_speed;                      /**< Default speed */
    char name[SD21_SERVO_NAME_LENGTH];          /**< Servomotor name */
} sd21_servo_t;

/**
 * @brief   SD21 configuration
 */
typedef struct {
    i2c_t i2c_dev_id;                       /**< I2C bus */
    uint16_t i2c_address;                   /**< I2C board address */
    i2c_speed_t i2c_speed_khz;              /**< I2C bus speed */

    uint8_t servos_nb;                      /**< number of servomotors */
    sd21_servo_t servos[SD21_SERVO_NUMOF];  /**< servomotors properties */
} sd21_conf_t;


/**
 * @brief Initialize SD21 board driver according to static configuration.
 *
 * @param[in]   sd21_config_new     SD21 configuration
 *
 * @return
 */
void sd21_init(const sd21_conf_t* sd21_config_new);

/**
 * @brief Servomotor driving function
 *
 * @param[in]   dev         SD21 device id
 * @param[in]   servo_id    Servomotor id
 * @param[in]   position    Servomotor position in ms
 *
 * @return                  0 on success
 *                          not 0 on failure
 */
int sd21_servo_control_position(sd21_t dev, uint8_t servo_id,
        uint16_t position);

/**
 * @brief Drive servomotor to given pre defined position
 *
 * @param[in]   dev         SD21 device id
 * @param[in]   servo_id    Servomotor id
 * @param[in]   pos_index   Servomotor pre defined position index
 *
 * @return                  0 on success
 *                          not 0 on failure
 */
int sd21_servo_reach_position(sd21_t dev, uint8_t servo_id,
        uint8_t pos_index);

/**
 * @brief Drive servomotor to reset position
 *
 * @param[in]   dev         SD21 device id
 * @param[in]   servo_id    Servomotor id
 *
 * @return                  0 on success
 *                          not 0 on failure
 */
int sd21_servo_reset_position(sd21_t dev, uint8_t servo_id);

/**
 * @brief Get SD21 firmware version.
 *
 * @param[in]   dev     SD21 device id
 *
 * @return              SD21 version number
 */
uint8_t sd21_get_version(sd21_t dev);

/**
 * @brief Get SD21 firmware version.
 *
 * @param[in]   dev     SD21 device id
 *
 * @return              SD21 battery voltage (V unit)
 */
double sd21_get_battery_voltage(sd21_t dev);

/**
 * @brief Get Servomotor current position.
 *
 * @param[in]   dev         SD21 device id
 * @param[in]   servo_id    Servomotor id
 * @param[out]  position    Servomotor position in ms
 *
 * @return                  0 on success
 *                          not 0 on failure
 */
int sd21_servo_get_position(sd21_t dev, uint8_t servo_id, uint16_t* position);

/**
 * @brief Get Servomotor name.
 *
 * @param[in]   dev         SD21 device id
 * @param[in]   servo_id    Servomotor id
 *
 * @return                  Servomotor name
 */
const char* sd21_servo_get_name(sd21_t dev, uint8_t servo_id);

/** @} */
