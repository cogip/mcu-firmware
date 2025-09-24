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
#include "board.h"
#include <malloc.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "board_internal.h"

/**
 * malloc() memory overhead (size + next address)
 */
#define MALLOC_OVERHEAD 8

/**
 * Pointer to start of the heap
 */
char* __sheap = 0;
/*
 * Pointer to end of the heap
 */
char* __eheap = 0;

/**
 * Board init function
 */
void board_init(void)
{
    /* Allocation status*/
    MALLINFO minfo = MALLINFO_FUNC();

    /* Compute heap start address */
    __sheap = (char*)sbrk(0) - minfo.uordblks - MALLOC_OVERHEAD;
    __eheap = (char*)sbrk(0);

    puts("COGIP native board initialized.");
}
