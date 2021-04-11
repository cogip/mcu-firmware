#pragma once

/* Standard includes */
#include <stdint.h>

/* Project includes */
#include "cogip_defs.h"

#define MAX_POINTS              256

#define GRAPH_MAX_VERTICES      64
#define DIJKSTRA_MAX_DISTANCE   13000000

#define AVOIDANCE_GRAPH_ERROR   -1


pose_t dijkstra(uint16_t target, uint16_t index);
pose_t avoidance(uint8_t index);
double distance_points(pose_t *a, pose_t *b);
int update_graph(const pose_t *s, const pose_t *f);
void init_polygons(void);
void build_avoidance_graph(void);
int add_polygon(polygon_t *polygon);
int add_dyn_polygon(polygon_t *polygon);
void reset_dyn_polygons(void);
uint8_t is_point_in_polygon(const polygon_t *polygons, pose_t p);
int8_t get_point_index_in_polygon(const polygon_t *polygons, pose_t p);
uint8_t is_segment_crossing_line(pose_t a, pose_t b, pose_t o, pose_t p);
uint8_t is_segment_crossing_segment(pose_t a, pose_t b, pose_t o, pose_t p);
uint8_t is_point_on_segment(pose_t a, pose_t b, pose_t o);
int check_polygon_collision(pose_t *point);
int avoidance_print_dyn_obstacles(int argc, char **argv);

static const polygon_t borders = {
    .points = {
        {
            .x = AVOIDANCE_BORDER_X_MIN,
            .y = AVOIDANCE_BORDER_Y_MIN
        },
        {
            .x = AVOIDANCE_BORDER_X_MAX,
            .y = AVOIDANCE_BORDER_Y_MIN
        },
        {
            .x = AVOIDANCE_BORDER_X_MAX,
            .y = AVOIDANCE_BORDER_Y_MAX
        },
        {
            .x = AVOIDANCE_BORDER_X_MIN,
            .y = AVOIDANCE_BORDER_Y_MAX
        },
    },
    .count = 4,
};
