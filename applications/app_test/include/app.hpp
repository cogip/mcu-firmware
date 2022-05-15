#pragma once

// Project includes
#include "app_borders.hpp"
#include "app_camp.hpp"
#include "path/Path.hpp"
#include "app_samples.hpp"
#include "app_shell.hpp"
#include "app_uartpb.hpp"

/*
 * Machine parameters
 */

#define USART_CONSOLE   USARTC0

/// Return the path of current application.
cogip::path::Path &app_get_path(void);

void app_init(void);
void app_init_tasks(void);
