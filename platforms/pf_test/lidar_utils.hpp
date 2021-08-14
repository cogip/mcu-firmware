#pragma once

/* System includes */
#include <inttypes.h>

/* Project includes */
#include "lds01.h"

lds01_t lidar_get_device(void);

void lidar_start(uint16_t max_distance, uint16_t min_intensity);
