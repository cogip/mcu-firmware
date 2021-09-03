/* Standard includes */
#include <math.h>
#include <stdio.h>

/* Project includes */
#include "app.hpp"
#include "avoidance.hpp"
#include "obstacles/obstacles.hpp"
#include "obstacles/List.hpp"
#include "platform.hpp"
#include "utils.h"
#include "cogip_defs/Coords.hpp"

#define START_INDEX     0
#define FINISH_INDEX    1

/* Array of valid points */
static cogip::cogip_defs::Pose _valid_points[GRAPH_MAX_VERTICES];
/* Number of valid points */
static uint8_t _valid_points_count = 0;

/**
 * Graph of valid segments. Each array entry is other valid
 * points accessibility mapped as a bitmap.
 * For instance:
 *   power of 2:         76543210
 *                       VVVVVVVV
 *   _graph[3] = 0x22 = 0b00100010
 *   Valid point 3 can reach valid point 1 and valid point 5
 */
static uint64_t _graph[GRAPH_MAX_VERTICES];

/* Start and finish poses */
static cogip::cogip_defs::Pose start_pose;
static cogip::cogip_defs::Pose finish_pose;

/**
 * Avoidance path with child direction hierarchy.
 * Starting index of this array is 0 which is the start pose. And value of
 * child array at this index is the index of the next path point. Last child
 * point is the final pose which has index 1.
 * For instance:
 *   _child[0]  = 12
 *   _child[12] = 8
 *   _child[8]  = 5
 *   _child[5]  = 22
 *   _child[22] = 1
 *
 * Path is then:
 *   _valid_points[0] (start_pose) -> _valid_points[12] -> _valid_points[8]
 *   -> _valid_points[5] -> _valid_points[22] -> _valid_points[1] (finish_pose)
 **/
static int _child[GRAPH_MAX_VERTICES];

/* Flag to indicate if a path has been successfully computed */
static bool _is_avoidance_computed = false;

/**
 * Validate each obstacle point. All valid points are a potential intermediate
 * pose of the avoidance path.
 */
static void _validate_obstacles_points(void)
{
    cogip::obstacles::List *obstacles_dyn = pf_get_dyn_obstacles();
    const cogip::obstacles::Polygon *borders = app_get_borders();

    /* For each obstacle */
    for (auto obstacle: *obstacles_dyn) {

        /* Check if obstacle center is inside borders */
        if (!borders->is_point_inside(obstacle->center())) {
            continue;
        }

        /* If that point is not in an other polygon, shape it in a bounding
         * box */
        if (!cogip::obstacles::is_point_in_obstacles(obstacle->center(), obstacle)) {

            cogip::cogip_defs::Polygon bb = obstacle->bounding_box(OBSTACLES_BB_NB_VERTICES,
                                                                   OBSTACLES_BB_RADIUS_MARGIN);

            /* Validate bounding box points */
            for (auto &point: bb) {
                if (borders->is_point_inside(point)
                    && (!cogip::obstacles::is_point_in_obstacles(point, nullptr))) {
                    _valid_points[_valid_points_count++] = { point.x(), point.y(), 0 };
                }
            }
        }
    }
}

/**
 * Build avoidance _graph.
 **/
static void _build_avoidance_graph(void)
{
    /* Validate all possible points */
    _validate_obstacles_points();

    /* Get all dynamic obstacles */
    cogip::obstacles::List *obstacles = pf_get_dyn_obstacles();

    /* For each segment of the valid points list */
    for (int p = 0; p < _valid_points_count; p++) {
        for (int p2 = p + 1; p2 < _valid_points_count; p2++) {
            bool collide = false;
            if (p != p2) {
                /* Check if that segment crosses a polygon */
                for (auto obstacle: *obstacles) {
                    if (obstacle->is_segment_crossing(_valid_points[p],
                                                      _valid_points[p2])) {
                        collide = true;
                        break;
                    }
                }
                /* If no collision, both points of the segment are added to
                 * the graph with distance between them */
                if ((p < GRAPH_MAX_VERTICES) && (p2 < GRAPH_MAX_VERTICES)) {
                    if (!collide) {
                        _graph[p] |= (1 << p2);
                        _graph[p2] |= (1 << p);
                    }
                    else {
                        _graph[p] &= ~(1 << p2);
                        _graph[p2] &= ~(1 << p);
                    }
                }
            }
        }
    }
}

/**
 * @Brief Dijkstra algorithm applied to the graph of valid segments.
 *
 * @return      graph resolution success
 */
static bool _dijkstra(void)
{
    /* List of graph vertices already checked */
    bool checked[GRAPH_MAX_VERTICES];
    /* Weight(distance) of each vertex */
    double distance[GRAPH_MAX_VERTICES];
    /* Graph vertices index */
    uint16_t v;
    /* Valid points index */
    uint16_t i;

    /* Dijkstra result path with parent direction hierarchy */
    int parent[GRAPH_MAX_VERTICES];

    /* Start pose */
    uint8_t start = START_INDEX;

    /* Initialize all arrays */
    for (int i = 0; i <= _valid_points_count; i++) {
        /* No point checked */
        checked[i] = false;
        /* All distances set to infinite */
        distance[i] = DIJKSTRA_MAX_DISTANCE;
        /* No parents */
        parent[i] = -1;
        /* No childs */
        _child[i] = -1;
    }

    /* Start point has a weight of 0 */
    distance[start] = 0;
    v = start;
    /* If start point has no reachable neighboor point exit on error */
    if (_graph[v] == 0) {
        _is_avoidance_computed = false;
        goto dijkstra_error_no_destination;
    }

    /* Loop until finish point is found or all points are checked */
    while ((v != FINISH_INDEX) && (checked[v] == false)) {
        /* Minimal distance found (initialized as infinite) */
        double min_distance = DIJKSTRA_MAX_DISTANCE;

        /* Current graph vertex is checked */
        checked[v] = true;

        /* Parse all valid points */
        for (i = 0; i < _valid_points_count; i++) {
            /* Check if the current valid point is one of the next vertices of
             * the graph. */
            if (_graph[v] & (1 << i)) {
                /* Compute edge weight between checked vertex and next one in
                 * graph */
                double weight = (_valid_points[v].x() - _valid_points[i].x());
                weight *= (_valid_points[v].x() - _valid_points[i].x());
                weight += (_valid_points[v].y() - _valid_points[i].y())
                          * (_valid_points[v].y() - _valid_points[i].y());
                weight = sqrt(weight);

                /* If the weight is not null and lower than the actual weight
                 * for this vertex, set its new minimal weight and set the
                 * current checked vertex as its new parent */
                if ((weight >= 0) && (distance[i] > (distance[v] + weight))) {
                    distance[i] = distance[v] + weight;
                    parent[i] = v;
                }
            }
        }

        /* Parse all valid points and select the one with the lowest weight to
         * be the next graph vertex checked */
        for (i = 1; i < _valid_points_count; i++) {
            if ((checked[i] == false) && (min_distance > distance[i])) {
                min_distance = distance[i];
                v = i;
            }
        }
    }

    /* Build child path from start to finish pose, by reversing parent path */
    i = 1;
    while (parent[i] >= 0) {
        _child[parent[i]] = i;
        i = parent[i];
    }

    /* Avoidance path is computed */
    _is_avoidance_computed = true;

dijkstra_error_no_destination:
    return _is_avoidance_computed;
}

/**
 * Return given index pose in the avoidance path.
 */
cogip::cogip_defs::Pose avoidance_get_pose(uint8_t index)
{
    /* Find n child in graph */
    uint8_t i = 0;
    uint8_t j = 0;

    while ((i != 1) && (j < index)) {
        i = _child[i];
        j++;
    }

    return _valid_points[i];
}

/**
 * Build avoidance graph.
 */
bool avoidance_build_graph(const cogip::cogip_defs::Pose &s, const cogip::cogip_defs::Pose &f)
{
    start_pose = s;
    finish_pose = f;
    cogip::obstacles::List *obstacles = pf_get_dyn_obstacles();
    const cogip::obstacles::Polygon *borders = app_get_borders();

    if (!borders->is_point_inside(finish_pose)) {
        _is_avoidance_computed = false;
        goto update_graph_error_finish_pose;
    }

    /* Check that start and destination point are not in an obstacle */
    for (auto obstacle: *obstacles) {
        if (obstacle->is_point_inside(finish_pose)) {
            goto update_graph_error_finish_pose;
        }
        if (obstacle->is_point_inside(start_pose)) {
            start_pose.set_coords(obstacle->nearest_point(start_pose));
        }
    }

    _valid_points_count = 0;
    _valid_points[_valid_points_count++] = start_pose;
    _valid_points[_valid_points_count++] = finish_pose;

    _build_avoidance_graph();

    _is_avoidance_computed = _dijkstra();

update_graph_error_finish_pose:
    return _is_avoidance_computed;
}

/**
 * Check if the given segment collides an obstacle.
 **/
bool avoidance_check_recompute(const cogip::cogip_defs::Pose &start,
                               const cogip::cogip_defs::Pose &stop)
{
    /* Get dynamic obstacle list */
    cogip::obstacles::List *obstacles = pf_get_dyn_obstacles();
    const cogip::obstacles::Polygon *borders = app_get_borders();

    /* Check if that segment crosses a polygon */
    for (auto obstacle: *obstacles) {

        /* Check if obstacle is inside borders */
        if (!borders->is_point_inside(obstacle->center())) {
            continue;
        }
        /* Check if start to finish pose segment is crossing an obtacle */
        if (obstacle->is_segment_crossing(start, stop)) {
            return true;
        }
    }

    return false;
}

/**
 * Print avoidance computed path
 */
void avoidance_print_path(cogip::tracefd::File &out)
{
    uint8_t i = 0;

    out.printf("[");
    /* Print only if avoidance has already been computed successfully */
    if (_is_avoidance_computed) {
        /* Print all intermediate poses */
        while (i != 1) {
            out.printf(
                "{\"x\":%.3lf,\"y\":%.3lf},",
                _valid_points[i].x(),
                _valid_points[i].y()
                );
            i = _child[i];
        }
        /* Print also finish pose */
        out.printf(
            "{\"x\":%.3lf,\"y\":%.3lf}",
            finish_pose.x(),
            finish_pose.y()
            );
    }
    out.printf("]");
}
