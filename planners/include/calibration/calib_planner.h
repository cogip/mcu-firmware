#pragma once

#include "ctrl/quadpid.h"

/**
 * @brief Initialize quadpid calibration, mainly adds commands to RIOT shell.
 *
 * @return
 */
void pln_calib_init(void);
