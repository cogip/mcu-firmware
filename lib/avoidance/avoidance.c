/* Standard includes */
#include <math.h>

/* Project includes */
#include "avoidance.h"
#include "collisions.h"
#include "obstacles.h"
#include "platform.h"
#include "utils.h"

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

/* List of visible points */
static pose_t valid_points[MAX_POINTS];
static uint8_t valid_points_count = 0;

static uint64_t graph[GRAPH_MAX_VERTICES];

static pose_t start_position = { .x = 0, .y = 0 };
static pose_t finish_position = { .x = 0, .y = 0 };

static int8_t _get_point_index_in_obstacle(const obstacle_t *obstacle, const pose_t *p)
{
    for (uint8_t i = 0; i < 4; i++) {
        if ((obstacle->points[i].x == p->x) && (obstacle->points[i].y == p->y)) {
            return i;
        }
    }
    return -1;
}

static pose_t _dijkstra(uint16_t target, uint16_t index)
{
    uint8_t checked[GRAPH_MAX_VERTICES];
    double distance[GRAPH_MAX_VERTICES];
    uint16_t v;
    int i;
    double weight;
    double min_distance;
    int parent[GRAPH_MAX_VERTICES];
    int child[GRAPH_MAX_VERTICES];
    /* TODO: start should be a parameter. More clean even if start is always index 0 in our case */
    int start = 0;

    for (int i = 0; i <= valid_points_count; i++) {
        checked[i] = FALSE;
        distance[i] = DIJKSTRA_MAX_DISTANCE;
        parent[i] = -1;
    }

    distance[start] = 0;
    v = start;
    if (graph[v] == 0) {
        goto dijkstra_error_no_destination;
    }

    while ((v != target) && (checked[v] == FALSE)) {
        min_distance = DIJKSTRA_MAX_DISTANCE;
        checked[v] = TRUE;
        for (i = 0; i < valid_points_count; i++) {
            if (graph[v] & (1 << i)) {
                weight = (valid_points[v].x - valid_points[i].x);
                weight *= (valid_points[v].x - valid_points[i].x);
                weight += (valid_points[v].y - valid_points[i].y)
                          * (valid_points[v].y - valid_points[i].y);
                weight = sqrt(weight);
                if ((weight >= 0) && (distance[i] > (distance[v] + weight))) {
                    distance[i] = distance[v] + weight;
                    parent[i] = v;
                }
            }
        }
        for (i = 1; i < valid_points_count; i++) {
            if ((checked[i] == FALSE) && (min_distance > distance[i])) {
                min_distance = distance[i];
                v = i;
            }
        }
    }

    /* Build reverse path (from start to finish) */
    i = 1;
    while (parent[i] >= 0) {
        child[parent[i]] = i;
        i = parent[i];
    }

    /* Find n child in graph */
    i = 0;
    v = 0;
    while ((i != 1) && (v < index)) {
        i = child[i];
        v++;
    }

    /* Return point */
    return valid_points[i];

dijkstra_error_no_destination:
    return start_position;
}

pose_t avoidance(uint8_t index)
{
    /* Build path graph */
    return _dijkstra(1, index);
}

int update_graph(const pose_t *s, const pose_t *f)
{
    start_position = *s;
    finish_position = *f;
    int index = 1;
    obstacles_t obstacles_id = pf_get_dyn_obstacles_id();

    if (!collisions_is_point_in_polygon(&finish_position, &borders)) {
        goto update_graph_error_finish_position;
    }

    /* Check that start and destination point are not in a polygon */
    for (size_t i = 0; i < obstacles_size(obstacles_id); i++) {
        const obstacle_t *obstacle = obstacles_get(obstacles_id, i);
        if (collisions_is_point_in_obstacle(obstacle, &finish_position)) {
            goto update_graph_error_finish_position;
        }
        if (collisions_is_point_in_obstacle(obstacle, &start_position)) {
            // find nearest polygon point
            double min = DIJKSTRA_MAX_DISTANCE;
            const pose_t *pose_tmp = &start_position;
            for (int j = 0; j < 4; j++) {
                if (!collisions_is_point_in_polygon(&obstacle->points[j], &borders)) {
                    continue;
                }

                double distance = collisions_distance_points(&start_position, &obstacle->points[j]);
                if (distance < min) {
                    min = distance;
                    pose_tmp = &obstacle->points[j];
                }
            }

            start_position = *pose_tmp;
            index = 0;
        }
    }

    valid_points_count = 0;
    valid_points[valid_points_count++] = start_position;
    valid_points[valid_points_count++] = finish_position;

    build_avoidance_graph();

    return index;

update_graph_error_finish_position:
    return AVOIDANCE_GRAPH_ERROR;
}

/* Build obstacle graph
 * Each obstacle is a polygon.
 * List all visible points: all points not contained in a polygon */
void build_avoidance_graph(void)
{
    obstacles_t obstacles_id = pf_get_dyn_obstacles_id();

    /* For each polygon */
    for (size_t i = 0; i < obstacles_size(obstacles_id); i++) {
        const obstacle_t *obstacle = obstacles_get(obstacles_id, i);
        /* and for each vertice of that polygon */
        for (int p = 0; p < 4; p++) {
            uint8_t collide = FALSE;
            /* Check if point is inside borders */
            if (!collisions_is_point_in_polygon(&obstacle->points[p], &borders)) {
                continue;
            }

            /* Check if this vertice is not inside an other polygon */
            for (size_t j = 0; j < obstacles_size(obstacles_id); j++) {
                const obstacle_t *obstacle2 = obstacles_get(obstacles_id, j);
                if (i == j) {
                    continue;
                }
                if (collisions_is_point_in_obstacle(obstacle2, &obstacle->points[p])) {
                    collide = TRUE;
                    break;
                }
            }
            /* If that point is not in an other polygon, add it to the list of valid points */
            if (!collide) {
                valid_points[valid_points_count++] = obstacle->points[p];
            }
        }
    }

    /* For each segment of the valid points list */
    for (int p = 0; p < valid_points_count; p++) {
        for (int p2 = p + 1; p2 < valid_points_count; p2++) {
            uint8_t collide = FALSE;
            if (p != p2) {
                /* Check if that segment crosses a polygon */
                for (size_t i = 0; i < obstacles_size(obstacles_id); i++) {
                    const obstacle_t *obstacle = obstacles_get(obstacles_id, i);
                    for (int v = 0; v < 4; v++) {
                        pose_t p_next = ((v == 3) ? obstacle->points[0] : obstacle->points[v + 1]);

                        if (collisions_is_segment_crossing_segment(&valid_points[p], &valid_points[p2], &obstacle->points[v], &p_next)) {
                            collide = TRUE;
                            break;
                        }
                        /* Special case of internal crossing of a polygon */
                        int8_t index = _get_point_index_in_obstacle(obstacle, &valid_points[p]);
                        int8_t index2 = _get_point_index_in_obstacle(obstacle, &valid_points[p2]);
                        if ((index == 0) && (index2 == 3)) {
                            continue;
                        }
                        if ((index2 == 0) && (index == 3)) {
                            continue;
                        }
                        if ((index >= 0) && (index2 >= 0) && (abs(index - index2) != 1)) {
                            collide = TRUE;
                            break;
                        }
                        if (collisions_is_point_on_segment(&valid_points[p], &valid_points[p2], &obstacle->points[v])) {
                            collide = TRUE;
                            break;
                        }
                    }
                    if (collide) {
                        break;
                    }
                }
                /* If no collision, both points of the segment are added to the graph with distance between them */
                if ((p < GRAPH_MAX_VERTICES) && (p2 < GRAPH_MAX_VERTICES)) {
                    if (!collide) {
                        graph[p] |= (1 << p2);
                        graph[p2] |= (1 << p);
                    }
                    else {
                        graph[p] &= ~(1 << p2);
                        graph[p2] &= ~(1 << p);
                    }
                }
            }
        }
    }
}
