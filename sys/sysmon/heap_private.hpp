///
/// Copyright (C) 2022 COGIP Robotics association <cogip35@gmail.com>
///
/// This file is subject to the terms and conditions of the GNU Lesser
/// General Public License v2.1. See the file LICENSE in the top level
/// directory for more details.

/// @defgroup    sys_sysmon_heap
/// @ingroup     sys_sysmon
/// @brief       Heap memory management
/// @{
///
/// @file
/// @brief       Heap memory management definitions
///              Mainly inspired from RIOT newlib syscall implementation file
///
/// @author      Gilles DOFFE <g.doffe@gmail.com>
///
/// @}

#pragma once

/// Heap description
typedef struct
{
    // cppcheck-suppress unusedStructMember
    char* start; ///< Heap start
    // cppcheck-suppress unusedStructMember
    char* end; ///< Heap end
} sysmon_heap_t;
