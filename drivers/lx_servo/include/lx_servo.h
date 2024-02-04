/*
 * Copyright (C) 2022 COGIP Robotics association <cogip35@gmail.com>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    driver_lx-servo LX servomotors driver
 * @ingroup     drivers_actuators
 * @brief       LX servomotors device driver
 *
 * LX servomotors uses one-wire serial half-duplex protocol.
 * LX-15D and LX-16A are well known servomotors working with this driver,
 * however this API should work with all LX HiWonder servomotors.
 *
 * This driver is inspired by Feetech driver written by
 * Loïc Dauphin <loic.dauphin@inria.fr>
 *
 * @{
 *
 * @file
 * @brief       LX servomotors interface API definition
 *
 * @author      Gilles DOFFE <g.doffe@gmail.com>
 */

#ifndef LX_SERVOS_H
#define LX_SERVOS_H

#include <stdbool.h>
#include <stdlib.h>

/* RIOT includes */
#include <uart_half_duplex.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef uint8_t lx_id_t;    /**< device id type */

#define LX_UART_BUFFER_SIZE 10

/**
 * @brief   Descriptor struct for a lx-servos device
 */
typedef struct {
    uart_half_duplex_t *stream; /**< the stream used */
    lx_id_t id;                 /**< the device address */
} lx_t;

/**
 * @brief   Serial communication errors
 */
typedef enum {
    LX_OK = 0,              /**< success */
    LX_TIMEOUT,             /**< no response from the device */
    LX_BUFFER_TOO_SMALL,    /**< buffer is too small for the message */
    LX_INVALID_MESSAGE,     /**< invalid message received */
    LX_INVALID_VALUE,       /**< invalid value received/written */
} lx_comm_error_t;

/**
 * @brief   Servomotor errors
 */
typedef enum {
    LX_SERVO_ERROR_OVER_TEMPERATURE = 1,    /**< over temperature */
    LX_SERVO_ERROR_OVER_VOLTAGE     = 2,    /**< over voltage */
    LX_SERVO_ERROR_STALLED          = 4     /**< servomotor is blocked */
} lx_error_t;

/**
 * @brief   Servomotor led status (on/off)
 */
typedef enum {
    LX_SERVO_LED_ON     = 0,    /**< led on */
    LX_SERVO_LED_OFF    = 1,    /**< led off */
} lx_led_status_t;

/**
 * @brief   Servomotor load modes
 */
typedef enum {
    LX_SERVO_UNLOADED   = 0,    /**< led on */
    LX_SERVO_LOADED     = 1,    /**< led off */
} lx_load_mode_t;

/**
 * @brief   Send a PING message to a device
 *
 * @param[in] device    the device address
 *
 * @return lx_comm_error_t
 *
 */
lx_comm_error_t lx_ping(lx_t *device);

/**
 * @brief   Initialize LX servomotor
 *
 * @param[out]  device  the LX device
 * @param[in]   stream  the half duplex stream
 * @param[in]   id      servomotor id
 */
void lx_init(lx_t *device, uart_half_duplex_t *stream, lx_id_t id);

/**
 * @brief   Go to the position in a given time
 *
 * @param[in]	device    the LX device
 * @param[in]	position  the target position
 * @param[in]	time      the movement time
 * @return lx_comm_error_t
 */
lx_comm_error_t lx_servo_move_time_write(const lx_t *device, const uint16_t position, const uint16_t time);

/**
 * @brief   Read last position/time order
 *
 * @param[in]	device    the LX device
 * @param[out]	position  the target position
 * @param[out]	time      the movement time
 * @return lx_comm_error_t
 */
lx_comm_error_t lx_servo_move_time_read(const lx_t *device, uint16_t *position, uint16_t *time);

/**
 * @brief   Register position and moving time
 *
 * @param[in]	device    the LX device
 * @param[in]	position  the target position
 * @param[in]	time      the movement time
 * @return lx_comm_error_t
 */
lx_comm_error_t lx_servo_move_time_wait_write(const lx_t *device, const uint16_t position, const uint16_t time);

/**
 * @brief Go to registered position
 *
 * @param[in]	device    the LX device
 * @return lx_comm_error_t
 */
lx_comm_error_t lx_servo_move_start(const lx_t *device);

/**
 * @brief Stop the motion
 *
 * @param[in]	device    the LX device
 * @return lx_comm_error_t
 */
lx_comm_error_t lx_servo_move_stop(const lx_t *device);

/**
 * @brief Set new servomotor ID
 *
 * @param[in]	device    the LX device
 * @param[in]	id        new ID (range 0 - 253)
 * @return lx_comm_error_t
 */
lx_comm_error_t lx_servo_id_write(const lx_t *device, const lx_id_t id);

/**
 * @brief Read servomotor ID
 *
 * @param[in]	device    the LX device
 * @param[out]	id        ID read
 * @return lx_comm_error_t
 */
lx_comm_error_t lx_servo_id_read(const lx_t *device, lx_id_t *id);

/**
 * @brief Set position offset (use lx_servo_position_offset_write() to save it permanently)
 *
 * @param[in]	device    the LX device
 * @param[out]	offset    position offset
 * @return lx_comm_error_t
 */
lx_comm_error_t lx_servo_position_offset_adjust(const lx_t *device, const int8_t offset);

/**
 * @brief Save position offset in memory
 *
 * @param[in]	device    the LX device
 * @return lx_comm_error_t
 */
lx_comm_error_t lx_servo_position_offset_write(const lx_t *device);

/**
 * @brief Read position offset from memory
 *
 * @param[in]	device    the LX device
 * @param[out]	offset    position offset
 * @return lx_comm_error_t
 */
lx_comm_error_t lx_servo_position_offset_read(const lx_t *device, int8_t *offset);

/**
 * @brief Set position min/max limits
 *
 * @param[in]	device        the LX device
 * @param[in]	position_min  minimum position limit
 * @param[in]	position_max  maximum position limit
 * @return lx_comm_error_t
 */
lx_comm_error_t lx_servo_position_limit_write(const lx_t *device, const uint16_t position_min, const uint16_t position_max);

/**
 * @brief Read position min/max limits
 *
 * @param[in]	device        the LX device
 * @param[out]	position_min  minimum position limit
 * @param[out]	position_max  maximum position limit
 * @return lx_comm_error_t
 */
lx_comm_error_t lx_servo_position_limit_read(const lx_t *device, uint16_t *position_min, uint16_t *position_max);

/**
 * @brief Set voltage min/max limits
 *
 * @param[in]	device    the LX device
 * @param[in]	vin_min   minimum voltage limit
 * @param[in]	vin_max   maximum voltage limit
 * @return lx_comm_error_t
 */
lx_comm_error_t lx_servo_vin_limit_write(const lx_t *device, const uint16_t vin_min, const uint16_t vin_max);

/**
 * @brief Read voltage min/max limits
 *
 * @param[in]	device    the LX device
 * @param[out]	vin_min   minimum voltage limit
 * @param[out]	vin_max   maximum voltage limit
 * @return lx_comm_error_t
 */
lx_comm_error_t lx_servo_vin_limit_read(const lx_t *device, uint16_t *vin_min, uint16_t *vin_max);

/**
 * @brief Set temperature max limit
 *
 * @param[in]	device    the LX device
 * @param[in]	temp_max  maximum temperature limit
 * @return lx_comm_error_t
 */
lx_comm_error_t lx_servo_temp_max_limit_write(const lx_t *device, const uint8_t temp_max);

/**
 * @brief Read temperature max limit
 *
 * @param[in]	device    the LX device
 * @param[out]	temp_max  maximum temperature limit
 * @return lx_comm_error_t
 */
lx_comm_error_t lx_servo_temp_max_limit_read(const lx_t *device, uint8_t *temp_max);

/**
 * @brief Read current temperature
 *
 * @param[in]	device    the LX device
 * @param[out]	temp      current temperature
 * @return lx_comm_error_t
 */
lx_comm_error_t lx_servo_temp_read(const lx_t *device, uint8_t *temp);

/**
 * @brief Read current voltage
 *
 * @param[in]	device    the LX device
 * @param[out]	vin       current voltage
 * @return lx_comm_error_t
 */
lx_comm_error_t lx_servo_vin_read(const lx_t *device, uint16_t *vin);

/**
 * @brief Read current position
 *
 * @param[in]	device    the LX device
 * @param[out]	pos       current position
 * @return lx_comm_error_t
 */
lx_comm_error_t lx_servo_pos_read(const lx_t *device, int16_t *pos);

/**
 * @brief Set servomotor or motor mode.
 * Set also the speed used only in motor mode.
 *
 * @param[in]	device    the LX device
 * @param[in]	mode      0 for servomotor mode, 1 for motor mode
 * @param[in]	speed     motor mode speed (range -1000 to 1000)
 * @return lx_comm_error_t
 */
lx_comm_error_t lx_servo_or_motor_mode_write(const lx_t *device, const bool mode, const int16_t speed);

/**
 * @brief Read servomotor mode and speed
 *
 * @param[in]	device    the LX device
 * @param[out]	mode      current motor mode
 * @param[out]	speed     current motor mode speed
 * @return lx_comm_error_t
 */
lx_comm_error_t lx_servo_or_motor_mode_read(const lx_t *device, bool *mode, int16_t *speed);

/**
 * @brief Set servomotor load
 *
 * @param[in]	device    the LX device
 * @param[in]	load_mode 0 for unloaded (no torque applied), 1 for loaded
 * @return lx_comm_error_t
 */
lx_comm_error_t lx_servo_load_or_unload_write(const lx_t *device, const lx_load_mode_t load_mode);

/**
 * @brief Read servomotor load state
 *
 * @param[in]	device    the LX device
 * @param[out]	load_mode 0 for unloaded (no torque applied), 1 for loaded
 * @return lx_comm_error_t
 */
lx_comm_error_t lx_servo_load_or_unload_read(const lx_t *device, lx_load_mode_t *load_mode);

/**
 * @brief Set status led on/off
 *
 * @param[in]	device    the LX device
 * @param[in]	led_ctrl  LX_SERVO_LED_ON led is on, LX_SERVO_LED_OFF led is off
 * @return lx_comm_error_t
 */
lx_comm_error_t lx_servo_led_ctrl_write(const lx_t *device, const lx_led_status_t led_ctrl);

/**
 * @brief Read status led configuration
 *
 * @param[in]	device    the LX device
 * @param[out]	led_ctrl  LX_SERVO_LED_ON led is on, LX_SERVO_LED_OFF led is off
 * @return lx_comm_error_t
 */
lx_comm_error_t lx_servo_led_ctrl_read(const lx_t *device, lx_led_status_t *led_ctrl);

/**
 * @brief Setup error led
 *
 * @param[in]	device    the LX device
 * @param[in]	led_error See lx_error_t
 * @return lx_comm_error_t
 */
lx_comm_error_t lx_servo_led_error_write(const lx_t *device, const lx_error_t led_error);

/**
 * @brief Read error led configuration
 *
 * @param[in]	device    the LX device
 * @param[out]	led_error See lx_error_t
 * @return lx_comm_error_t
 */
lx_comm_error_t lx_servo_led_error_read(const lx_t *device, lx_error_t *led_error);

#ifdef __cplusplus
}
#endif

#endif /* LX_SERVOS_H */
/** @} */
