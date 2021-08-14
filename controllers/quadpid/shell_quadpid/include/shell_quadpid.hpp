#pragma once

#include "quadpid.hpp"

/**
 * @brief Initialize quadpid controller calibration shell
 *
 * Register quadpid controller calibration commands.
 */
void ctrl_quadpid_shell_init(ctrl_quadpid_t *ctrl_quadpid_new);
