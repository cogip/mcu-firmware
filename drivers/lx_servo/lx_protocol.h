/*
 * Copyright (C) 2022 COGIP Robotics association <cogip35@gmail.com>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    driver_lx_servo LX servomotors serial communication protocol
 * @ingroup     drivers
 * @brief       LX servomotors serial communication protocol
 *
 * LX servomotors use a one-wire serial half-duplex protocol.
 * Each packet is formatted as following:
 *  _________________________________________________________________________________
 * |   HEADER  | SERVO_ID |     DATA_LENGTH     |  COMMAND   | PARAMETERS | CHECKSUM |
 * | (2 bytes) | (1 byte) |       (1 byte)      |  (1 byte)  |  (X bytes) | (1 byte) |
 * |---------------------------------------------------------------------------------|
 * |           | 0 - 254  | 1 + Command length  |            |  PARAM_1 + |          |
 * | 0x55 0x55 |          | + Parameters length | Command ID |  ......... |   (2)    |
 * |           |   (1)    | + Checksum length   |            |  + PARAM_N |          |
 *  ---------------------------------------------------------------------------------
 * (1) Servo ID 254 is reserved to broadcast packet
 * (2) CHECKSUM = ~(SERVO_ID + DATA_LENGTH + PARAM_1 + ... + PARAM_N) & 0xFF
 *
 * There are two kinds of commands:
 *  - Read commands
 *  - Write commands
 *
 * Read commands are executed in 2 steps:
 *  - A write on serial bus (TX) to indicate to the servomotor which parameter to read.
 *  - A read on serial bus (RX) to get the servomotor answer.
 *
 * As the bus is a one-wire serial half-duplex bus, read commands cannot be performed in
 * broadcast.
 *
 * @{
 *
 * @file
 * @brief       LX servomotors private protocol API
 *
 * @author      Gilles DOFFE <g.doffe@gmail.com>
 */

#ifndef LX_PROTOCOL_H
#define LX_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#define LX_START                        (0x55)  /**< packet header */

#define LX_BROADCAST_ID                 254     /**< broadcast ID for write
                                                   commands only */

#define LX_PACKET_HEADER_SIZE           3       /**< packet header size
                                                   (HEADER + SERVO_ID) */

#define LX_PACKET_PAYLOAD_HEADER_SIZE   3       /**< packet payload header size
                                                   (DATA_LENGTH + COMMAND
                                                 + CHECKSUM) */

#define LX_PACKET_PAYLOAD_SIZE(len) \
    (LX_PACKET_PAYLOAD_HEADER_SIZE + len)       /**< packet full payload size
                                                   (DATA_LENGTH + COMMAND +
                                                   CHECKSUM + PARAMETERS) */

#define LX_PACKET_SIZE(len) \
    (LX_PACKET_HEADER_SIZE \
     + LX_PACKET_PAYLOAD_SIZE(len))             /**< full packet size */

/**
 * @brief Available baudrates for LX servomotors
 */
typedef enum {
    LX_B_115200 = 0,    /**< 115200 bauds */
} lx_baudrate_t;

#define LX_SERVO_POSITION_LIMIT_MAX     1000    /**< maximum position */
#define LX_SERVO_LED_ERROR_LIMIT_MAX    7       /**< maximum led error value */
#define LX_SERVO_OFFSET_LIMIT_MIN       -125    /**< minimum position offset */
#define LX_SERVO_OFFSET_LIMIT_MAX       125     /**< maximum position offset */
#define LX_SERVO_SPEED_LIMIT_MIN        -1000   /**< minimum speed */
#define LX_SERVO_SPEED_LIMIT_MAX        1000    /**< maximum speed */
#define LX_SERVO_TIME_LIMIT_MAX         30000   /**< maximum move duration */
#define LX_SERVO_TEMP_MAX_LIMIT_MIN     50      /**< minimum temperature */
#define LX_SERVO_TEMP_MAX_LIMIT_MAX     100     /**< maximum temperature */
#define LX_SERVO_VIN_LIMIT_MIN          4500    /**< minimum supply voltage */
#define LX_SERVO_VIN_LIMIT_MAX          12000   /**< maximum supply voltage */

typedef enum {
    LX_SERVO_MOVE_TIME_WRITE        = 1,    /**< go to position in a given time */
    LX_SERVO_MOVE_TIME_WAIT_WRITE   = 7,    /**< register position & move time */
    LX_SERVO_MOVE_START             = 11,   /**< go to registered position */
    LX_SERVO_MOVE_STOP              = 12,   /**< stop the motion */
    LX_SERVO_ID_WRITE               = 13,   /**< set new servomotor ID */
    LX_SERVO_ANGLE_OFFSET_ADJUST    = 17,   /**< set position offset */
    LX_SERVO_ANGLE_OFFSET_WRITE     = 18,   /**< save position offset */
    LX_SERVO_ANGLE_LIMIT_WRITE      = 20,   /**< set position min/max limits */
    LX_SERVO_VIN_LIMIT_WRITE        = 22,   /**< set voltage min/max limits */
    LX_SERVO_TEMP_MAX_LIMIT_WRITE   = 24,   /**< set temperature max limit */
    LX_SERVO_OR_MOTOR_MODE_WRITE    = 29,   /**< set servomotor or speed mode */
    LX_SERVO_LOAD_OR_UNLOAD_WRITE   = 31,   /**< set servomotor load */
    LX_SERVO_LED_CTRL_WRITE         = 33,   /**< set status led (on/off) */
    LX_SERVO_LED_ERROR_WRITE        = 35,   /**< setup error led */
} lx_write_command_t;

typedef enum {
    LX_SERVO_MOVE_TIME_READ         = 2,    /**< read last position/time order */
    LX_SERVO_MOVE_TIME_WAIT_READ    = 8,    /**< NOT IMPLEMENTED ON SERVOMOTOR */
    LX_SERVO_ID_READ                = 14,   /**< read servomotor ID */
    LX_SERVO_ANGLE_OFFSET_READ      = 19,   /**< read position offset */
    LX_SERVO_ANGLE_LIMIT_READ       = 21,   /**< read position min/max limits */
    LX_SERVO_VIN_LIMIT_READ         = 23,   /**< read voltage min/max limits */
    LX_SERVO_TEMP_MAX_LIMIT_READ    = 25,   /**< read temperature max limit */
    LX_SERVO_TEMP_READ              = 26,   /**< read current temperature */
    LX_SERVO_VIN_READ               = 27,   /**< read current voltage */
    LX_SERVO_POS_READ               = 28,   /**< read current position */
    LX_SERVO_OR_MOTOR_MODE_READ     = 30,   /**< read servomotor mode */
    LX_SERVO_LOAD_OR_UNLOAD_READ    = 32,   /**< read servomotor load state */
    LX_SERVO_LED_CTRL_READ          = 34,   /**< read status led configuration */
    LX_SERVO_LED_ERROR_READ         = 36,   /**< read error led configuration */
} lx_read_command_t;


#ifdef __cplusplus
}
#endif

#endif /* LX_PROTOCOL_H */
/** @} */
