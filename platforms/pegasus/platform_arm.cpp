/*
 * Copyright (C) 2022 COGIP Robotics association <cogip35@gmail.com>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/* Firmware includes */
#include "board.h"
#include "lx_servo.h"
#include "vacuum_pump_params.h"

/* RIOT includes */
#include <periph/gpio.h>

#ifndef LX_DIR_PIN
#define LX_DIR_PIN  GPIO_UNDEF
#endif

/* Half duplex UART stream */
static uart_half_duplex_t pf_lx_stream;

/* LX servos command buffer */
static uint8_t pf_lx_servos_buffer[LX_UART_BUFFER_SIZE];

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

static void pf_lx_half_duplex_uart_init(void) {
    uart_half_duplex_params_t params = {
        .uart = UART_DEV(LX_UART_DEV),
        .baudrate = 115200,
        .dir = { dir_init, dir_enable_tx, dir_disable_tx },
    };

    int ret = uart_half_duplex_init(&pf_lx_stream, pf_lx_servos_buffer, ARRAY_SIZE(pf_lx_servos_buffer), &params);

    if (ret == UART_HALF_DUPLEX_NODEV) {
        puts("Error: invalid UART device given");
    }
    else if (ret == UART_HALF_DUPLEX_NOBAUD) {
        puts("Error: given baudrate is not applicable");
    }
    else if (ret == UART_HALF_DUPLEX_INTERR) {
        puts("Error: internal error");
    }
    else if (ret == UART_HALF_DUPLEX_NOMODE) {
        puts("Error: given mode is not applicable");
    }
    else if (ret == UART_HALF_DUPLEX_NOBUFF) {
        puts("Error: invalid buffer given");
    }
    else {
        printf("Successfully initialized LX Servos TTL bus UART_DEV(%i)\n", params.uart);
    }
}

 uart_half_duplex_t* pf_lx_get_stream(void) {
     return &pf_lx_stream;
 }

static void pf_vacuum_pumps_init(void) {
    for (uint8_t i = 0; i < VACUUM_PUMP_NUMOF; i++) {
        vacuum_pump_init(i, &vacuum_pump_params[i]);
    }
}

void pf_arms_init(void) {
    pf_lx_half_duplex_uart_init();
    pf_vacuum_pumps_init();
}