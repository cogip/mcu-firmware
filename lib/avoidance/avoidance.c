/* Standard includes */
#include <math.h>
#include <stdio.h>

/* Project includes */
#include "avoidance.h"
#include "collisions.h"
#include "obstacles.h"
#include "platform.h"
#include "utils.h"

static const obstacle_t borders = {
    .type = OBSTACLE_POLYGON,
    .form.polygon = {
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
    }
};


/* List of visible points */
static pose_t valid_points[MAX_POINTS];
static uint8_t valid_points_count = 0;

static uint64_t graph[GRAPH_MAX_VERTICES];

static pose_t start_position = { .coords.x = 0, .coords.y = 0 };
static pose_t finish_position = { .coords.x = 0, .coords.y = 0 };

static int child[GRAPH_MAX_VERTICES];

static bool avoidance_computed_flag = false;

static pose_t _dijkstra(uint16_t target, uint16_t index)
{
    bool checked[GRAPH_MAX_VERTICES];
    double distance[GRAPH_MAX_VERTICES];
    uint16_t v;
    int i;
    double weight;
    double min_distance;
    int parent[GRAPH_MAX_VERTICES];
    /* TODO: start should be a parameter. More clean even if start is always index 0 in our case */
    int start = 0;

    for (int i = 0; i <= valid_points_count; i++) {
        checked[i] = false;
        distance[i] = DIJKSTRA_MAX_DISTANCE;
        parent[i] = -1;
    }

    distance[start] = 0;
    v = start;
    if (graph[v] == 0) {
        goto dijkstra_error_no_destination;
    }

    while ((v != target) && (checked[v] == false)) {
        min_distance = DIJKSTRA_MAX_DISTANCE;
        checked[v] = true;
        for (i = 0; i < valid_points_count; i++) {
            if (graph[v] & (1 << i)) {
                weight = (valid_points[v].coords.x - valid_points[i].coords.x);
                weight *= (valid_points[v].coords.x - valid_points[i].coords.x);
                weight += (valid_points[v].coords.y - valid_points[i].coords.y)
                          * (valid_points[v].coords.y - valid_points[i].coords.y);
                weight = sqrt(weight);
                if ((weight >= 0) && (distance[i] > (distance[v] + weight))) {
                    distance[i] = distance[v] + weight;
                    parent[i] = v;
                }
            }
        }
        for (i = 1; i < valid_points_count; i++) {
            if ((checked[i] == false) && (min_distance > distance[i])) {
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

    avoidance_computed_flag = true;

    /* Return point */
    return valid_points[i];

dijkstra_error_no_destination:
    avoidance_computed_flag = false;
    return start_position;
}

void avoidance_print_path(FILE *out)
{
    int i = 0;

    fprintf(out, "[");
    fflush(out);
    if (avoidance_computed_flag) {
        while (i != 1) {
            fprintf(out, "{\"x\": %lf, \"y\": %lf},",
                    valid_points[i].coords.x,
                    valid_points[i].coords.y
                    );
            i = child[i];
        }
        fprintf(out, "{\"x\": %lf, \"y\": %lf}",
                finish_position.coords.x,
                finish_position.coords.y
                );
    }
    fprintf(out, "]");
    fflush(out);
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

    if (!obstacles_is_point_in_obstacle(&borders, &finish_position.coords)) {
        goto update_graph_error_finish_position;
    }

    /* Check that start and destination point are not in an obstacle */
    for (size_t i = 0; i < obstacles_get_nb_obstacles(obstacles_id); i++) {
        const obstacle_t *obstacle = obstacles_get(obstacles_id, i);
        if (obstacles_is_point_in_obstacle(obstacle, &finish_position.coords)) {
            goto update_graph_error_finish_position;
        }
        if (obstacles_is_point_in_obstacle(obstacle, &start_position.coords)) {
            start_position.coords = obstacles_find_nearest_point_in_obstacle(obstacle,
                                                                             &start_position.coords);
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

void validate_obstacles(void)
{
    obstacles_dyn_id_t obstacles_dyn_id = pf_get_dyn_obstacles_id();

    /* For each obstacle */
    for (size_t i = 0; i < obstacles_get_nb_obstacles(obstacles_dyn_id); i++) {
        const obstacle_t *obstacle = obstacles_get(obstacles_dyn_id, i);

        /* Check if point is inside borders */
        if (!obstacles_is_point_in_obstacle(&borders, &obstacle->center)) {
            continue;
        }

        /* If that point is not in an other polygon, shape it in a bounding
         * box
         */
        if (!obstacles_is_point_in_obstacles(&obstacle->center, obstacle)) {

            polygon_t bb = obstacles_compute_obstacle_bounding_box(obstacle,
                                                                   OBSTACLES_BB_NB_VERTICES,
                                                                   OBSTACLES_BB_RADIUS_MARGIN);

            /* Validate bounding box */
            for (uint8_t j = 0; j < bb.count; j++) {
                if ((obstacles_is_point_in_obstacle(&borders, &bb.points[j]))
                    && (!obstacles_is_point_in_obstacles(&bb.points[j],
                                                         NULL))) {
                    valid_points[valid_points_count++] = (pose_t){
                        bb.points[j], 0
                    };
                }
            }
        }
    }
}

/* Build obstacle graph
 * Each obstacle is a polygon.
 * List all visible points: all points not contained in a polygon */
bool is_colliding(const pose_t *start, const pose_t *stop)
{
    obstacles_t obstacles_id = pf_get_dyn_obstacles_id();

    /* Check if that segment crosses a polygon */
    for (size_t i = 0; i < obstacles_get_nb_obstacles(obstacles_id); i++) {
        const obstacle_t *obstacle = obstacles_get(obstacles_id, i);

        /* Check if obstacle  is inside borders */
        if (!obstacles_is_point_in_obstacle(&borders, &obstacle->center)) {
            continue;
        }
        if (obstacles_is_segment_crossing_obstacle(&start->coords, &stop->coords, obstacle)) {
            return true;
        }
    }

    return false;
}

/* Build obstacle graph
 * Each obstacle is a polygon.
 * List all visible points: all points not contained in a polygon */
void build_avoidance_graph(void)
{
    validate_obstacles();

    obstacles_t obstacles_id = pf_get_dyn_obstacles_id();

    /* For each segment of the valid points list */
    for (int p = 0; p < valid_points_count; p++) {
        for (int p2 = p + 1; p2 < valid_points_count; p2++) {
            bool collide = false;
            if (p != p2) {
                /* Check if that segment crosses a polygon */
                for (size_t i = 0; i < obstacles_get_nb_obstacles(obstacles_id); i++) {
                    const obstacle_t *obstacle = obstacles_get(obstacles_id, i);
                    if (obstacles_is_segment_crossing_obstacle(&valid_points[p].coords,
                                                               &valid_points[p2].coords, obstacle)) {
                        collide = true;
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
