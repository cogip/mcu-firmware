/*
 * Copyright (C) 2021 COGIP Robotics association <cogip35@gmail.com>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    platforms_shell Common platform shell
 * @ingroup     platforms
 * @brief       Common platform shell with shell management
 *
 * The common platform shell code defines shell commands to retrieve
 * informations from the platform component which has a central role.
 * For now the shell has the following commands:
 * * _state         : Print current state
 * * mt             : Test all DC motors
 *
 * @{
 * @file
 * @brief       Common platform shell headers
 *
 * @author      Eric Courtois <eric.courtois@gmail.com>
 * @author      Gilles DOFFE <g.doffe@gmail.com>
 */

void pf_shell_init(void);

/** @} */
