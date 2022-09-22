/**
 * Native Board board_init implementation
 *
 * Copyright (C) 2014 Ludwig Knüpfer <ludwig.knuepfer@fu-berlin.de>
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
#include <stdio.h>
#include <stdlib.h>
#include "board.h"

#include "board_internal.h"


/**
 * Nothing to initialize at the moment.
 */
void board_init(void)
{
    puts("COGIP native board initialized.");
}


