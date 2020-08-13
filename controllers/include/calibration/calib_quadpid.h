#pragma once

#include "ctrl/quadpid.h"

/**
 * @brief Initialize quadpid calibration, mainly adds commands to RIOT shell.
 *
 * @return
 */
void ctrl_quadpid_calib_init(void);
