#pragma once

// Project includes
#include "obstacles/Polygon.hpp"
#include "path/Path.hpp"

/*
 * Machine parameters
 */

#define USART_CONSOLE   USARTC0

/// Return a polygon obstacle delimitng the table borders.
const cogip::obstacles::Polygon &app_get_borders(void);

/// Return the path of current application.
cogip::path::Path &app_get_path(void);

void app_init(void);
void app_init_tasks(void);

/// Run Application wizard
void app_wizard(void);
