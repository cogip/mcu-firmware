/*
 * Copyright (C) 2022 COGIP Robotics association <cogip35@gmail.com>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/* Firmware includes */
#include "lx_servo.h"
#include "shell.h"
#include "stdio_uart.h"

/* System includes */
#include <stdio.h>
#include <string.h>

#ifndef LX_DIR_PIN
#define LX_DIR_PIN  GPIO_UNDEF
#endif

static void dir_init(uart_t uart)
{
    (void)uart;
    gpio_init(LX_DIR_PIN, GPIO_OUT);
}

static void dir_enable_tx(uart_t uart)
{
    (void)uart;
    gpio_set(LX_DIR_PIN);
}

static void dir_disable_tx(uart_t uart)
{
    (void)uart;
    gpio_clear(LX_DIR_PIN);
}

static const int32_t baudrates[] = {
    115200L,
};

static uint8_t lx_servos_buffer[10];
static uart_half_duplex_t stream;

static int parse_uart(char *arg)
{
    unsigned uart = atoi(arg);

    if (uart >= UART_NUMOF) {
        printf("Error: Invalid UART_DEV device specified (%u).\n", uart);
        return -1;
    }
    else if (UART_DEV(uart) == STDIO_UART_DEV) {
        printf("Error: The selected UART_DEV(%u) is used for the shell!\n", uart);
        return -2;
    }
    return uart;
}

static int32_t parse_baud(char *arg)
{
    int32_t baud = atoi(arg);

    for (size_t i = 0; i < ARRAY_SIZE(baudrates); i++) {
        if (baud == baudrates[i]) {
            return baud;
        }
    }

    printf("Error: Invalid baudrate (%s)\n", arg);
    return -1;
}

static int parse_dev(char *arg)
{
    int dev = atoi(arg);

    if (dev < 0 || 254 < dev) {
        printf("Error: Invalid device id (%s)\n", arg);
        return -1;
    }
    return dev;
}

static int parse_angle(char *arg)
{
    int angle = atoi(arg);

    if (angle < 0 || 1000 < angle) {
        printf("Error: Invalid angle (%s)\n", arg);
        return -1;
    }
    return angle;
}

static int parse_time(char *arg)
{
    int time = atoi(arg);

    if (time < 0 || 30000 < time) {
        printf("Error: Invalid time (%s)\n", arg);
        return -1;
    }
    return time;
}

static int parse_u8(char *arg)
{
    int u8 = atoi(arg);

    if (u8 < 0 || 7 < u8) {
        printf("Error: Invalid unsigned char (%s)\n", arg);
        return -1;
    }
    return u8;
}

static int cmd_init(int argc, char **argv)
{
    int uart = -1;
    int baud = -1;
    uint32_t timeout = -1;

    if (argc != 3 && argc != 4) {
        printf("usage: %s <uart> <baudrate> [<timeout_us>]\n", argv[0]);
        puts("available baudrates :");
        for (size_t i = 0; i < ARRAY_SIZE(baudrates); i++) {
            printf("\t%ld\n", (long int)baudrates[i]);
        }
        return 1;
    }
    /* parse parameters */
    uart = parse_uart(argv[1]);
    if (uart < 0) {
        return -1;
    }

    baud = parse_baud(argv[2]);
    if (baud < 0) {
        return -1;
    }

    if (argc == 4) {
        timeout = (uint32_t)atol(argv[3]);
        if (timeout == 0) {
            printf("Error : Invalid timeout (%s)", argv[3]);
            return -1;
        }
    }

    /* init */
    uart_half_duplex_params_t params = {
        .uart = uart,
        .baudrate = baud,
        .dir = { dir_init, dir_enable_tx, dir_disable_tx },
    };

    int ret = uart_half_duplex_init(&stream, lx_servos_buffer, ARRAY_SIZE(lx_servos_buffer), &params);

    if (argc == 4) {
        stream.timeout_us = timeout;
    }

    if (ret == UART_HALF_DUPLEX_NODEV) {
        puts("Error: invalid UART device given");
        return -1;
    }
    if (ret == UART_HALF_DUPLEX_NOBAUD) {
        puts("Error: given baudrate is not applicable");
        return -1;
    }
    if (ret == UART_HALF_DUPLEX_INTERR) {
        puts("Error: internal error");
        return -1;
    }
    if (ret == UART_HALF_DUPLEX_NOMODE) {
        puts("Error: given mode is not applicable");
        return -1;
    }
    if (ret == UART_HALF_DUPLEX_NOBUFF) {
        puts("Error: invalid buffer given");
        return -1;
    }

    printf("Successfully initialized LX Servos TTL bus UART_DEV(%i)\n", uart);
    return 0;
}

static int cmd_write_id(int argc, char **argv)
{
    int id = -1;
    int new_id = -1;
    lx_t device;

    if (argc != 3) {
        printf("usage: %s <dev_id> <dev_new_id>\n", argv[0]);
        return 1;
    }
    /* parse parameters */
    id = parse_dev(argv[1]);
    new_id = parse_dev(argv[2]);
    if ((id < 0) || (new_id < 0)) {
        return -1;
    }

    device.stream = &stream;
    device.id = id;

    /* ping */
    if (lx_servo_id_write(&device, new_id) == LX_OK) {
        printf("Device id %i change request successfully sent to: %i\n", id, new_id);
    }
    else {
        printf("No response from device %i\n", id);
    }
    return 0;
}

static int cmd_load_on(int argc, char **argv)
{
    int id = -1;
    lx_t device;

    if (argc != 2) {
        printf("usage: %s <dev_id>\n", argv[0]);
        return 1;
    }
    /* parse parameters */
    id = parse_dev(argv[1]);
    if (id < 0) {
        return -1;
    }

    device.stream = &stream;
    device.id = id;

    /* Turn led on */
    if (lx_servo_load_or_unload_write(&device, LX_SERVO_LOADED) == LX_OK) {
        printf("Device id %i led has been loaded on\n", id);
    }
    else {
        printf("No response from device %i\n", id);
    }
    return 0;
}

static int cmd_load_off(int argc, char **argv)
{
    int id = -1;
    lx_t device;

    if (argc != 2) {
        printf("usage: %s <dev_id>\n", argv[0]);
        return 1;
    }
    /* parse parameters */
    id = parse_dev(argv[1]);
    if (id < 0) {
        return -1;
    }

    device.stream = &stream;
    device.id = id;

    /* Turn led on */
    if (lx_servo_load_or_unload_write(&device, LX_SERVO_UNLOADED) == LX_OK) {
        printf("Device id %i led has been unloaded on\n", id);
    }
    else {
        printf("No response from device %i\n", id);
    }
    return 0;
}

static int cmd_led_on(int argc, char **argv)
{
    int id = -1;
    lx_t device;

    if (argc != 2) {
        printf("usage: %s <dev_id>\n", argv[0]);
        return 1;
    }
    /* parse parameters */
    id = parse_dev(argv[1]);
    if (id < 0) {
        return -1;
    }

    device.stream = &stream;
    device.id = id;

    /* Turn led on */
    if (lx_servo_led_ctrl_write(&device, LX_SERVO_LED_ON) == LX_OK) {
        printf("Device id %i led has been turned on\n", id);
    }
    else {
        printf("No response from device %i\n", id);
    }
    return 0;
}

static int cmd_led_off(int argc, char **argv)
{
    int id = -1;
    lx_t device;

    if (argc != 2) {
        printf("usage: %s <dev_id>\n", argv[0]);
        return 1;
    }
    /* parse parameters */
    id = parse_dev(argv[1]);
    if (id < 0) {
        return -1;
    }

    device.stream = &stream;
    device.id = id;

    /* Turn led on */
    if (lx_servo_led_ctrl_write(&device, LX_SERVO_LED_OFF) == LX_OK) {
        printf("Device id %i led has been turned off\n", id);
    }
    else {
        printf("No response from device %i\n", id);
    }
    return 0;
}

static int cmd_led_conf(int argc, char **argv)
{
    int id = -1;
    int led_conf;
    lx_t device;

    if (argc != 3) {
        printf("usage: %s <dev_id>\n", argv[0]);
        return 1;
    }
    /* parse parameters */
    id = parse_dev(argv[1]);
    if (id < 0) {
        return -1;
    }
    led_conf = parse_u8(argv[2]);
    if (led_conf < 0) {
        return -1;
    }

    device.stream = &stream;
    device.id = id;

    /* Turn led on */
    if (lx_servo_led_error_write(&device, led_conf) == LX_OK) {
        printf("Device id %i error led has been configured\n", id);
    }
    else {
        printf("No response from device %i\n", id);
    }
    return 0;
}

static int cmd_move_time(int argc, char **argv)
{
    int id = -1;
    int angle = -1;
    int time = -1;
    lx_t device;

    if (argc != 4) {
        printf("usage: %s <dev_id> <angle> <time>\n", argv[0]);
        puts("<angle> is between 0 and 1000, <time> is between 0 and 30000.");
        return 1;
    }
    /* parse parameters */
    id = parse_dev(argv[1]);
    angle = parse_angle(argv[2]);
    time = parse_time(argv[3]);
    if ((id < 0) || (angle < 0) || (time < 0)) {
        return -1;
    }

    device.stream = &stream;
    device.id = id;

    if (lx_servo_move_time_write(&device, angle, time) == LX_OK) {
        printf("Position %i change request successfully sent to device %i in %i.\n", angle, id, time);
    }
    else {
        printf("No response from device %i\n", id);
    }
    return 0;
}

static int cmd_move(int argc, char **argv)
{
    int id = -1;
    int angle = -1;
    lx_t device;

    if (argc != 3) {
        printf("usage: %s <dev_id> <angle>\n", argv[0]);
        puts("<angle> is between 0 and 1000.");
        return 1;
    }
    /* parse parameters */
    id = parse_dev(argv[1]);
    angle = parse_angle(argv[2]);
    if ((id < 0) || (angle < 0)) {
        return -1;
    }

    device.stream = &stream;
    device.id = id;

    if (lx_servo_move_time_write(&device, angle, 0) == LX_OK) {
        printf("Position %i change request successfully sent to device %i.\n", angle, id);
    }
    else {
        printf("No response from device %i\n", id);
    }
    return 0;
}

static int cmd_move_time_wait(int argc, char **argv)
{
    int id = -1;
    int angle = -1;
    int time = -1;
    lx_t device;

    if (argc != 4) {
        printf("usage: %s <dev_id> <angle> <time>\n", argv[0]);
        puts("<angle> is between 0 and 1000, <time> is between 0 and 30000.");
        return 1;
    }
    /* parse parameters */
    id = parse_dev(argv[1]);
    angle = parse_angle(argv[2]);
    time = parse_time(argv[3]);
    if ((id < 0) || (angle < 0) || (time < 0)) {
        return -1;
    }

    device.stream = &stream;
    device.id = id;

    /* ping */
    if (lx_servo_move_time_wait_write(&device, angle, time) == LX_OK) {
        printf("Position %i change request successfully sent to device %i in %i\n", angle, id, time);
    }
    else {
        printf("No response from device %i\n", id);
    }
    return 0;
}

static int cmd_move_wait(int argc, char **argv)
{
    int id = -1;
    int angle = -1;
    lx_t device;

    if (argc != 3) {
        printf("usage: %s <dev_id> <angle>\n", argv[0]);
        puts("<angle> is between 0 and 1000.");
        return 1;
    }
    /* parse parameters */
    id = parse_dev(argv[1]);
    angle = parse_angle(argv[2]);
    if ((id < 0) || (angle < 0)) {
        return -1;
    }

    device.stream = &stream;
    device.id = id;

    if (lx_servo_move_time_wait_write(&device, angle, 0) == LX_OK) {
        printf("Position %i change request successfully sent to device %i.\n", angle, id);
    }
    else {
        printf("No response from device %i\n", id);
    }
    return 0;
}

static int cmd_move_start(int argc, char **argv)
{
    lx_t device;

    if (argc < 2) {
        printf("usage: %s <dev_id>\n", argv[0]);
        return 1;
    }

    for (uint8_t i = 1; i < argc; i++) {
        /* parse parameters */
        int id = parse_dev(argv[i]);
        if (id < 0) {
            return -1;
        }

        lx_init(&device, &stream, id);

        /* ping */
        if (lx_servo_move_start(&device) == LX_OK) {
            printf("Device %i move started\n", id);
        }
        else {
            printf("Device %i move cannot be started, use 'move_time_wait' first\n", id);
        }
    }

    return LX_OK;
}

static int cmd_ping(int argc, char **argv)
{
    int id = -1;
    lx_t device;

    if (argc != 2) {
        printf("usage: %s <dev_id>\n", argv[0]);
        return 1;
    }
    /* parse parameters */
    id = parse_dev(argv[1]);
    if (id < 0) {
        return -1;
    }

    lx_init(&device, &stream, id);

    /* ping */
    if (lx_ping(&device) == LX_OK) {
        printf("Device %i responded\n", id);
    }
    else {
        printf("No response from device %i\n", id);
    }
    return 0;
}

static int cmd_scan(int argc, char **argv)
{
    int min = -1;
    int max = -1;
    lx_t device;

    if (argc == 3) {
        min = atoi(argv[1]);
        max = atoi(argv[2]);
        if (min < 0) {
            return -1;
        }
        if (max > 254) {
            return -1;
        }
        if (max < min) {
            return -1;
        }
    }
    else if (argc == 1) {
        min = 0;
        max = 254;
    }
    else {
        printf("usage: %s [<min_id> <max_id>]\n", argv[0]);
        return 1;
    }

    lx_init(&device, &stream, 0);

    /* ping */
    puts("Scanning...");
    for (int id = min; id < max; id++) {
        device.id = id;
        if (lx_ping(&device) == LX_OK) {
            printf("Device %i available\n", id);
        }
    }
    puts("End");
    return 0;
}

static int cmd_status(int argc, char **argv)
{
    int id = -1;
    int ret = -1;
    uint16_t angle = 0;
    uint16_t angle_min = 0;
    uint16_t angle_max = 0;
    int8_t angle_offset = 0;
    lx_led_status_t led;
    lx_error_t led_status;
    lx_load_mode_t load = 0;
    bool mode = 0;
    int16_t pos = 0;
    int16_t speed = 0;
    uint8_t temp = 0;
    uint8_t temp_max_limit = 0;
    uint16_t time = 0;
    uint16_t vin = 0;
    uint16_t vin_min = 0;
    uint16_t vin_max = 0;
    lx_t device;

    if (argc != 2) {
        printf("usage: %s <dev_id>\n", argv[0]);
        return -1;
    }
    /* parse parameters */
    id = parse_dev(argv[1]);
    if (id < 0) {
        return -1;
    }

    lx_init(&device, &stream, id);

    /* SERVO_ID_READ */
    ret = lx_servo_id_read(&device, (uint8_t *)&id);
    if (ret) {
        printf("ID - Error %d\n", ret);
    }
    else {
        printf("ID                                          = %d\n", id);
    }

    /* SERVO_MOVE_TIME_READ */
    ret = lx_servo_move_time_read(&device, &angle, &time);
    if (ret) {
        printf("MOVE_TIME - Error %d\n", ret);
    }
    else {
        printf("MOVE_TIME - angle                           = %u\n", angle);
        printf("MOVE_TIME - time                            = %u\n", time);
    }

    /* SERVO_ANGLE_OFFSET_READ */
    ret = lx_servo_position_offset_read(&device, &angle_offset);
    if (ret) {
        printf("ANGLE_OFFSET - Error %d\n", ret);
    }
    else {
        printf("ANGLE_OFFSET                                = %d\n", angle_offset);
    }

    /* SERVO_ANGLE_LIMIT_READ */
    ret = lx_servo_position_limit_read(&device, &angle_min, &angle_max);
    if (ret) {
        printf("ANGLE_LIMIT - Error %d\n", ret);
    }
    else {
        printf("ANGLE_LIMIT_min                             = %u\n", angle_min);
        printf("ANGLE_LIMIT_max                             = %u\n", angle_max);
    }

    /* SERVO_VIN_LIMIT_READ */
    ret = lx_servo_vin_limit_read(&device, &vin_min, &vin_max);
    if (ret) {
        printf("VIN_LIMIT - Error %d\n", ret);
    }
    else {
        printf("VIN_LIMIT_min                               = %u\n", vin_min);
        printf("VIN_LIMIT_max                               = %u\n", vin_max);
    }

    /* LX_SERVO_TEMP_MAX_LIMIT_READ */
    ret = lx_servo_temp_max_limit_read(&device, &temp_max_limit);
    if (ret) {
        printf("TEMP_MAX_LIMIT - Error %d\n", ret);
    }
    else {
        printf("TEMP_MAX_LIMIT                              = %u\n", temp_max_limit);
    }

    /* LX_SERVO_TEMP_READ */
    ret = lx_servo_temp_read(&device, &temp);
    if (ret) {
        printf("TEMP - Error %d\n", ret);
    }
    else {
        printf("TEMP                                        = %u\n", temp);
    }

    /* LX_SERVO_VIN_READ */
    ret = lx_servo_vin_read(&device, &vin);
    if (ret) {
        printf("VIN - Error %d\n", ret);
    }
    else {
        printf("VIN                                         = %u\n", vin);
    }

    /* LX_SERVO_POS_READ */
    ret = lx_servo_pos_read(&device, &pos);
    if (ret) {
        printf("POS - Error %d\n", ret);
    }
    else {
        printf("POS                                         = %d\n", pos);
    }

    /* LX_SERVO_OR_MOTOR_MODE_READ */
    ret = lx_servo_or_motor_mode_read(&device, &mode, &speed);
    if (ret) {
        printf("SERVO_OR_MOTOR_MODE - Error %d\n", ret);
    }
    else {
        printf("SERVO_OR_MOTOR_MODE - mode                  = %s\n", (mode == 0 ? "Servo" : "Motor"));
        printf("SERVO_OR_MOTOR_MODE      - speed            = %d\n", speed);
    }

    /* LX_SERVO_LOAD_OR_UNLOAD_READ */
    ret = lx_servo_load_or_unload_read(&device, &load);
    if (ret) {
        printf("LOAD_OR_UNLOAD - Error %d\n", ret);
    }
    else {
        printf("LOAD_OR_UNLOAD                              = %s\n", (mode == 0 ? "Unload" : "Load"));
    }

    /* LX_SERVO_LED_CTRL_READ */
    ret = lx_servo_led_ctrl_read(&device, &led);
    if (ret) {
        printf("LX_SERVO_LED_CTRL - Error %d\n", ret);
    }
    else {
        printf("LX_SERVO_LED_CTRL                           = %s\n", (led == 0 ? "ON" : "OFF"));
    }

    /* LX_SERVO_LED_ERROR_READ */
    ret = lx_servo_led_error_read(&device, &led_status);
    if (ret) {
        printf("LX_SERVO_LED_ERROR - Error %d\n", ret);
    }
    else {
        printf("LX_SERVO_LED_ERROR - Over temperature       = %s\n", ((led_status & LX_SERVO_ERROR_OVER_TEMPERATURE) ? "Yes" : "No"));
        printf("LX_SERVO_LED_ERROR - Over voltage           = %s\n", ((led_status & LX_SERVO_ERROR_OVER_VOLTAGE) ? "Yes" : "No"));
        printf("LX_SERVO_LED_ERROR - Stalled                = %s\n", ((led_status & LX_SERVO_ERROR_STALLED) ? "Yes" : "No"));
    }

    return LX_OK;
}

static const shell_command_t shell_commands[] = {
    { "init", "Initialize a LX TTL bus with a given baudrate", cmd_init },
    { "ping", "Ping a LX device", cmd_ping },
    { "scan", "Find all LX devices between min_id and max_id", cmd_scan },
    { "status", "Display all readable properties of a device", cmd_status },
    { "id", "Set new id for an LX device", cmd_write_id },
    { "ld", "Load on", cmd_load_on },
    { "uld", "Load off", cmd_load_off },
    { "lc", "Status led configuration", cmd_led_conf },
    { "lon", "Turn led on", cmd_led_on },
    { "loff", "Turn led off", cmd_led_off },
    { "m", "Move to new position at full speed", cmd_move },
    { "mw", "Register new position at full speed", cmd_move_wait },
    { "mt", "Move to new position in the given time", cmd_move_time },
    { "mtw", "Register new position to reach in the given time", cmd_move_time_wait },
    { "ms", "Move to new position set by 'mtw' or 'mw'", cmd_move_start },
    { NULL, NULL, NULL }
};

int main(void)
{
    puts("\nManual LX device driver test");
    puts("===================================");
    puts("This application is intended for testing LX TTL bus\n");

    char line_buf[SHELL_DEFAULT_BUFSIZE];

    shell_run(shell_commands, line_buf, SHELL_DEFAULT_BUFSIZE);

    return 0;
}
