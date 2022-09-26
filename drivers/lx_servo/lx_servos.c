/*
 * Copyright (C) 2022 COGIP Robotics association <cogip35@gmail.com>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_lx_servos
 * @{
 *
 * @file
 * @brief       Driver implementation for LX-Servos devices
 *
 * @author      Gilles DOFFE <g.doffe@gmail.com>
 *
 * @}
 */

#define ENABLE_DEBUG (0)
#include <debug.h>

#include <lx_servo.h>

#include "lx_protocol.h"
#include "lx_reader.h"
#include "lx_writer.h"

#include "periph/uart.h"
#include "byteorder.h"

#include <string.h>

#define set_move_time_wait_write(x) ((move_time_wait_check[x / 8] |= 0x1 << (x % 8)))
#define check_move_time_wait_write(x) ((move_time_wait_check[x / 8] >> (x % 8)) & 0x1)

static uint8_t move_time_wait_check[32];

void lx_init(lx_t *device, uart_half_duplex_t *stream, lx_id_t id)
{
    device->stream = stream;
    device->id = id;
}

lx_comm_error_t lx_ping(lx_t *device)
{
    lx_id_t id = 0;

    return lx_servo_id_read(device, &id);
}

static int lx_write(const lx_t *device, lx_write_command_t command, const uint8_t *data, size_t length)
{
    uart_half_duplex_set_tx(device->stream);
    if (device->stream->size < LX_PACKET_SIZE(length)) {
        DEBUG("%s: Buffer too small on device %u!\n", __func__, device->id);
        return LX_BUFFER_TOO_SMALL;
    }

    lx_writer_t pw;

    lx_writer_init(&pw, device->stream->buffer, device->stream->size);
    lx_writer_write_make(&pw, device->id, command, data, length);
    if (uart_half_duplex_send(device->stream, pw.size) != pw.size) {
        DEBUG("%s: Timeout error on device %u!\n", __func__, device->id);
        return LX_TIMEOUT;
    }

    return LX_OK;
}

static int lx_read(const lx_t *device, lx_read_command_t command, uint8_t *data, size_t length)
{
    uart_half_duplex_set_tx(device->stream);
    if (device->stream->size < LX_PACKET_SIZE(length)) {
        DEBUG("%s: Buffer too small on device %u!\n", __func__, device->id);
        return LX_BUFFER_TOO_SMALL;
    }

    lx_writer_t pw;

    lx_writer_init(&pw, device->stream->buffer, device->stream->size);
    lx_writer_read_make(&pw, device->id, command);
    uart_half_duplex_send(device->stream, pw.size);

    uart_half_duplex_set_rx(device->stream);
    const size_t esize = LX_PACKET_SIZE(length);

    if (uart_half_duplex_recv(device->stream, esize) != esize) {
        DEBUG("%s: Timeout error on device %u!\n", __func__, device->id);
        return LX_TIMEOUT;
    }

    lx_reader_t pr;

    lx_reader_init(&pr, device->stream->buffer, esize);

    if (!lx_reader_is_valid(&pr)) {
        DEBUG("%s: Invalid message from device %u!\n", __func__, device->id);
        return LX_INVALID_MESSAGE;
    }

    if (lx_reader_response_get_payload_size(&pr) != LX_PACKET_PAYLOAD_SIZE(length)) {
        DEBUG("%s: Invalid payload from device %u!\n", __func__, device->id);
        return LX_INVALID_MESSAGE;
    }

    memcpy(data, lx_reader_response_get_payload(&pr), length);

    return LX_OK;
}

lx_comm_error_t lx_servo_move_time_write(const lx_t *device, const uint16_t position,
                                         const uint16_t time)
{
    uint8_t data[4];

    if (position > LX_SERVO_POSITION_LIMIT_MAX) {
        return LX_INVALID_VALUE;
    }

    data[0] = (uint8_t)(position & 0xFF);
    data[1] = (uint8_t)((position >> 8) & 0xFF);
    data[2] = (uint8_t)(time & 0xFF);
    data[3] = (uint8_t)((time >> 8) & 0xFF);

    return lx_write(device, LX_SERVO_MOVE_TIME_WRITE, data, 4);
}

lx_comm_error_t lx_servo_move_time_read(const lx_t *device, uint16_t *position, uint16_t *time)
{
    uint8_t data[4];

    lx_comm_error_t ret = lx_read(device, LX_SERVO_MOVE_TIME_READ, data, 4);

    if (ret == LX_OK) {
        *position = data[0] + (data[1] << 8);
        *time = data[2] + (data[3] << 8);
    }

    return ret;
}

lx_comm_error_t lx_servo_move_time_wait_write(const lx_t *device, const uint16_t position,
                                              const uint16_t time)
{
    uint8_t data[4];
    lx_comm_error_t res = LX_INVALID_MESSAGE;

    if (position > LX_SERVO_POSITION_LIMIT_MAX) {
        return LX_INVALID_VALUE;
    }

    data[0] = (uint8_t)(position & 0xFF);
    data[1] = (uint8_t)((position >> 8) & 0xFF);
    data[2] = (uint8_t)(time & 0xFF);
    data[3] = (uint8_t)((time >> 8) & 0xFF);

    res = lx_write(device, LX_SERVO_MOVE_TIME_WAIT_WRITE, data, 4);

    if (res == LX_OK) {
        set_move_time_wait_write(device->id);
    }

    return res;
}

lx_comm_error_t lx_servo_move_start(const lx_t *device)
{
    if (check_move_time_wait_write(device->id)) {
        return lx_write(device, LX_SERVO_MOVE_START, NULL, 0);
    }
    else {
        return LX_INVALID_MESSAGE;
    }
}

lx_comm_error_t lx_servo_move_stop(const lx_t *device)
{
    return lx_write(device, LX_SERVO_MOVE_STOP, NULL, 0);
}

lx_comm_error_t lx_servo_id_write(const lx_t *device, const lx_id_t id)
{
    return lx_write(device, LX_SERVO_ID_WRITE, &id, 1);
}

lx_comm_error_t lx_servo_id_read(const lx_t *device, lx_id_t *id)
{
    return lx_read(device, LX_SERVO_ID_READ, id, 1);
}

lx_comm_error_t lx_servo_position_offset_adjust(const lx_t *device, const int8_t offset)
{
    if ((offset > LX_SERVO_OFFSET_LIMIT_MAX) || (offset < LX_SERVO_OFFSET_LIMIT_MIN)) {
        return LX_INVALID_VALUE;
    }

    return lx_write(device, LX_SERVO_ANGLE_OFFSET_ADJUST, (uint8_t *)&offset, 1);
}

lx_comm_error_t lx_servo_position_offset_write(const lx_t *device)
{
    return lx_write(device, LX_SERVO_ANGLE_OFFSET_WRITE, NULL, 0);
}

lx_comm_error_t lx_servo_position_offset_read(const lx_t *device, int8_t *offset)
{
    return lx_read(device, LX_SERVO_ANGLE_OFFSET_READ, (uint8_t *)offset, 1);
}

lx_comm_error_t lx_servo_position_limit_write(const lx_t *device, const uint16_t position_min,
                                              const uint16_t position_max)
{
    uint8_t data[4];

    if ((position_max > LX_SERVO_POSITION_LIMIT_MAX) || (position_min >= position_max)) {
        return LX_INVALID_VALUE;
    }

    data[0] = (uint8_t)(position_min & 0xFF);
    data[1] = (uint8_t)((position_min >> 8) & 0xFF);
    data[2] = (uint8_t)(position_max & 0xFF);
    data[3] = (uint8_t)((position_max >> 8) & 0xFF);

    return lx_write(device, LX_SERVO_ANGLE_LIMIT_WRITE, data, 4);
}

lx_comm_error_t lx_servo_position_limit_read(const lx_t *device, uint16_t *position_min, uint16_t *position_max)
{
    uint8_t data[4];

    lx_comm_error_t ret = lx_read(device, LX_SERVO_ANGLE_LIMIT_READ, data, 4);

    if (ret == LX_OK) {
        *position_min = data[0] + (data[1] << 8);
        *position_max = data[2] + (data[3] << 8);
    }

    return ret;
}

lx_comm_error_t lx_servo_vin_limit_write(const lx_t *device, const uint16_t vin_min,
                                         const uint16_t vin_max)
{
    uint8_t data[4];

    if ((vin_max > LX_SERVO_VIN_LIMIT_MAX) || (vin_min >= vin_max) || (vin_min < LX_SERVO_VIN_LIMIT_MIN)) {
        return LX_INVALID_VALUE;
    }

    data[0] = (uint8_t)(vin_min & 0xFF);
    data[1] = (uint8_t)((vin_min >> 8) & 0xFF);
    data[2] = (uint8_t)(vin_max & 0xFF);
    data[3] = (uint8_t)((vin_max >> 8) & 0xFF);

    return lx_write(device, LX_SERVO_VIN_LIMIT_WRITE, data, 4);
}

lx_comm_error_t lx_servo_vin_limit_read(const lx_t *device, uint16_t *vin_min, uint16_t *vin_max)
{
    uint8_t data[4];

    lx_comm_error_t ret = lx_read(device, LX_SERVO_VIN_LIMIT_READ, data, 4);

    if (ret == LX_OK) {
        *vin_min = data[0] + (data[1] << 8);
        *vin_max = data[2] + (data[3] << 8);
    }

    return ret;
}

lx_comm_error_t lx_servo_temp_max_limit_write(const lx_t *device, const uint8_t temp_max)
{
    if ((temp_max > LX_SERVO_TEMP_MAX_LIMIT_MAX) || (temp_max < LX_SERVO_TEMP_MAX_LIMIT_MIN)) {
        return LX_INVALID_VALUE;
    }

    return lx_write(device, LX_SERVO_TEMP_MAX_LIMIT_WRITE, &temp_max, 1);
}

lx_comm_error_t lx_servo_temp_max_limit_read(const lx_t *device, uint8_t *temp_max)
{
    return lx_read(device, LX_SERVO_TEMP_MAX_LIMIT_READ, temp_max, 1);
}

lx_comm_error_t lx_servo_temp_read(const lx_t *device, uint8_t *temp)
{
    return lx_read(device, LX_SERVO_TEMP_READ, temp, 1);
}

lx_comm_error_t lx_servo_vin_read(const lx_t *device, uint16_t *vin)
{
    uint8_t data[2];

    lx_comm_error_t ret = lx_read(device, LX_SERVO_VIN_READ, data, 2);

    if (ret == LX_OK) {
        *vin = data[0] + (data[1] << 8);
    }

    return ret;
}

lx_comm_error_t lx_servo_pos_read(const lx_t *device, int16_t *pos)
{
    uint8_t data[2];

    lx_comm_error_t ret = lx_read(device, LX_SERVO_POS_READ, data, 2);

    if (ret == LX_OK) {
        *pos = data[0] | (data[1] << 8);
    }

    return ret;
}

lx_comm_error_t lx_servo_or_motor_mode_write(const lx_t *device, const bool mode,
                                             const int16_t speed)
{
    uint8_t data[4];

    if ((speed > LX_SERVO_SPEED_LIMIT_MAX) || (speed < LX_SERVO_SPEED_LIMIT_MIN)) {
        return LX_INVALID_VALUE;
    }

    data[0] = (uint8_t)(mode);
    data[1] = 0;
    data[2] = (uint8_t)(speed & 0xFF);
    data[3] = (uint8_t)(((uint16_t)speed >> 8) & 0xFF);

    return lx_write(device, LX_SERVO_OR_MOTOR_MODE_WRITE, data, 4);
}

lx_comm_error_t lx_servo_or_motor_mode_read(const lx_t *device, bool *mode, int16_t *speed)
{
    uint8_t data[4];

    lx_comm_error_t ret = lx_read(device, LX_SERVO_OR_MOTOR_MODE_READ, data, 4);

    if (ret == LX_OK) {
        *mode = data[0];
        *speed = data[2] + (data[3] << 8);
    }

    return ret;
}

lx_comm_error_t lx_servo_load_or_unload_write(const lx_t *device, const lx_load_mode_t load_mode)
{
    return lx_write(device, LX_SERVO_LOAD_OR_UNLOAD_WRITE, (const uint8_t *)&load_mode, 1);
}

lx_comm_error_t lx_servo_load_or_unload_read(const lx_t *device, lx_load_mode_t *load_mode)
{
    return lx_read(device, LX_SERVO_LOAD_OR_UNLOAD_READ, (uint8_t *)load_mode, 1);
}

lx_comm_error_t lx_servo_led_ctrl_write(const lx_t *device, const lx_led_status_t led_ctrl)
{
    return lx_write(device, LX_SERVO_LED_CTRL_WRITE, (const uint8_t *)&led_ctrl, 1);
}

lx_comm_error_t lx_servo_led_ctrl_read(const lx_t *device, lx_led_status_t *led_ctrl)
{
    return lx_read(device, LX_SERVO_LED_CTRL_READ, (uint8_t *)led_ctrl, 1);
}

lx_comm_error_t lx_servo_led_error_write(const lx_t *device, const lx_error_t led_error)
{
    if (led_error > LX_SERVO_LED_ERROR_LIMIT_MAX) {
        return LX_INVALID_VALUE;
    }

    return lx_write(device, LX_SERVO_LED_ERROR_WRITE, &led_error, 1);
}

lx_comm_error_t lx_servo_led_error_read(const lx_t *device, lx_error_t *led_error)
{
    return lx_read(device, LX_SERVO_LED_ERROR_READ, led_error, 1);
}
