#pragma once

/* Standard includes */
#include <stdint.h>

/* Project includes */
#include "cogip_defs.h"

#define MAX_POINTS              256

#define GRAPH_MAX_VERTICES      64
#define DIJKSTRA_MAX_DISTANCE   13000000

#define AVOIDANCE_GRAPH_ERROR   -1

pose_t avoidance(uint8_t index);
int update_graph(const pose_t *s, const pose_t *f);
void build_avoidance_graph(void);
