#pragma once

#include "planners/Planner.hpp"

/**
 * @brief Initialize quadpid shell, mainly adds commands to RIOT shell.
 * @param[in] pln planner object
 * @return
 */
void pln_shell_init(cogip::planners::Planner *pln);
