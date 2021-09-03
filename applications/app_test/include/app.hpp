#pragma once

// Project includes
#include "obstacles.hpp"

/*
 * Machine parameters
 */

#define USART_CONSOLE   USARTC0

/// @brief Return a polygon obstacle delimitng the table borders.
/// @return                      table borders
const cogip::obstacles::Polygon *app_get_borders(void);

void app_init(void);
void app_init_tasks(void);
