/*
 * Copyright (C) 2018 COGIP Robotics association
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     boards_cogip2018-cortex
 * @{
 *
 * @file
 * @brief       Board initialization code for cogip2018-cortex
 *
 * @author      Gilles DOFFE <g.doffe@gmail.com>
 *
 * @}
 */

#include "board.h"
#include "periph/gpio.h"

void board_init(void)
{
    /* initialize the CPU */
    cpu_init();
}
