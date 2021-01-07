/*
 * Copyright (C) 2021 COGIP Robotics association <cogip35@gmail.com>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    platforms_calibration Common platform calibration
 * @ingroup     platforms
 * @brief       Common platform calibration with shell management
 *
 * The common platform calibration code defines shell commands to retrieve
 * informations from the platform component which has a central role.
 * For now the shell has the following commands:
 * * _dyn_obstacles : Print dynamic obstacles
 * * _help_json     : Display available commands in JSON format
 * * _pose          : Print current pose
 * * _set_shm_key   : Set shared memory key to communicate with simulator
 *
 * @{
 * @file
 * @brief       QuadPID controllers API and datas
 *
 * @author      Gilles DOFFE <g.doffe@gmail.com>
 * @author      Yannick GICQUEL <yannick.gicquel@gmail.com>
 * @author      Stephen CLYMANS <sclymans@gmail.com>
 */
/* Project includes */
#include "ctrl.h"

void pf_init_calib_tasks(ctrl_t* pf_ctrl);
void pf_calib_init(void);
