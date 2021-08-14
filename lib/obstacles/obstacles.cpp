/* System includes */
#include <math.h>
#include <string.h>

/* Project includes */
#include "collisions.h"
#include "obstacles.h"
#include "trigonometry.h"

#define ENABLE_DEBUG (0)
#include "debug.h"

/**
 * @brief   Obstacles context
 */
typedef struct {
    obstacles_params_t params;                  /**< parameters */
    size_t nb_obstacles;                        /**< number of obstacles */
    obstacle_t obstacles[OBSTACLES_MAX_NUMBER]; /**< obstacles list */
    mutex_t lock;                               /**< obstacles context protection mutex */
} obstacles_context_t;

/* Allocate memory for the obstacles contexts */
obstacles_context_t obstacles_contexts[OBSTACLES_NUMOF];

/* Number of initialized obstacles */
obstacles_t obstacles_initialized = 0;

obstacles_context_t obstacles_contexts[OBSTACLES_NUMOF];

obstacles_t obstacles_init(const obstacles_params_t *obstacles_params)
{
    assert(obstacles_initialized < OBSTACLES_NUMOF);
    obstacles_t id = obstacles_initialized;
    obstacles_context_t *obstacles_context = &obstacles_contexts[id];
    obstacles_context->params = *obstacles_params;
    obstacles_context->nb_obstacles = 0;
    mutex_init(&obstacles_context->lock);
    obstacles_initialized++;
    return id;
}

obstacle_t obstacles_rectangle_init(coords_t center, double length_x,
                                    double length_y, double angle)
{
    obstacle_t rect = {
        .type = OBSTACLE_RECTANGLE,
        .center = center,
        .radius = sqrt(length_x * length_x + length_y * length_y) / 2,
        .angle = angle,
        .form.rectangle = {
            .length_x = length_x,
            .length_y = length_y,
            .points = {
                (coords_t){
                    .x = (center.x - length_x / 2) * cos(DEG2RAD(angle)) -
                         (center.y - length_y / 2) * sin(DEG2RAD(angle)),
                    .y = (center.x - length_x / 2) * sin(DEG2RAD(angle)) +
                         (center.y - length_y / 2) * cos(DEG2RAD(angle)),
                },
                (coords_t){
                    .x = (center.x - length_x / 2) * cos(DEG2RAD(angle)) -
                         (center.y + length_y / 2) * sin(DEG2RAD(angle)),
                    .y = (center.x - length_x / 2) * sin(DEG2RAD(angle)) +
                         (center.y + length_y / 2) * cos(DEG2RAD(angle)),
                },
                (coords_t){
                    .x = (center.x + length_x / 2) * cos(DEG2RAD(angle)) -
                         (center.y + length_y / 2) * sin(DEG2RAD(angle)),
                    .y = (center.x + length_x / 2) * sin(DEG2RAD(angle)) +
                         (center.y + length_y / 2) * cos(DEG2RAD(angle)),
                },
                (coords_t){
                    .x = (center.x + length_x / 2) * cos(DEG2RAD(angle)) -
                         (center.y - length_y / 2) * sin(DEG2RAD(angle)),
                    .y = (center.x + length_x / 2) * sin(DEG2RAD(angle)) +
                         (center.y - length_y / 2) * cos(DEG2RAD(angle)),
                },
            }
        }
    };

    return rect;
}

polygon_t obstacles_compute_obstacle_bounding_box(const obstacle_t *obstacle,
                                                  const uint8_t nb_vertices,
                                                  const double radius_margin)
{
    double radius = obstacle->radius * (1 + radius_margin);

    polygon_t bb;

    bb.count = (nb_vertices < 4) ? 4 : nb_vertices;

    for (uint8_t i = 0; i < bb.count; i++) {
        bb.points[i].x = obstacle->center.x +
                         radius * cos(((double)i * 2 * M_PI) / (double)bb.count);
        bb.points[i].y = obstacle->center.y +
                         radius * sin(((double)i * 2 * M_PI) / (double)bb.count);
    }

    return bb;
}

double obstacles_compute_radius(const obstacle_t *obstacle)
{
    /* Temporary object */
    polygon_t polygon_tmp;

    switch (obstacle->type) {
        case OBSTACLE_CIRCLE:
            return obstacle->form.circle.radius;
        case OBSTACLE_RECTANGLE:
            polygon_tmp.count = 4;
            memcpy(polygon_tmp.points, obstacle->form.rectangle.points,
                   polygon_tmp.count * sizeof(coords_t));
            return collisions_compute_polygon_radius(&polygon_tmp, &obstacle->center);
        case OBSTACLE_POLYGON:
            return collisions_compute_polygon_radius(&obstacle->form.polygon, &obstacle->center);
        default:
            DEBUG("obstacle: Wrong type, should never happen, return false)");
    }

    return 0;
}

double obstacles_get_default_circle_radius(const obstacles_t obstacles_id)
{
    assert(obstacles_id < OBSTACLES_NUMOF);
    const obstacles_context_t *obstacles_context = &obstacles_contexts[obstacles_id];
    return obstacles_context->params.default_circle_radius;
}

uint32_t obstacles_get_min_distance(const obstacles_t obstacles_id)
{
    assert(obstacles_id < OBSTACLES_NUMOF);
    const obstacles_context_t *obstacles_context = &obstacles_contexts[obstacles_id];
    return obstacles_context->params.min_distance;
}

uint32_t obstacles_get_max_distance(const obstacles_t obstacles_id)
{
    assert(obstacles_id < OBSTACLES_NUMOF);
    const obstacles_context_t *obstacles_context = &obstacles_contexts[obstacles_id];
    return obstacles_context->params.max_distance;
}

size_t obstacles_get_nb_obstacles(const obstacles_t obstacles_id)
{
    assert(obstacles_id < OBSTACLES_NUMOF);
    const obstacles_context_t *obstacles_context = &obstacles_contexts[obstacles_id];
    return obstacles_context->nb_obstacles;
}

void obstacles_reset(const obstacles_t obstacles_id)
{
    assert(obstacles_id < OBSTACLES_NUMOF);
    obstacles_context_t *obstacles_context = &obstacles_contexts[obstacles_id];
    obstacles_context->nb_obstacles = 0;
}

const obstacle_t *obstacles_get(const obstacles_t obstacles_id, size_t n)
{
    assert(obstacles_id < OBSTACLES_NUMOF);
    const obstacles_context_t *obstacles_context = &obstacles_contexts[obstacles_id];
    if (n >= obstacles_context->nb_obstacles) {
        return NULL;
    }
    return &obstacles_context->obstacles[n];
}

const obstacle_t *obstacles_get_all(const obstacles_t obstacles_id)
{
    assert(obstacles_id < OBSTACLES_NUMOF);
    const obstacles_context_t *obstacles_context = &obstacles_contexts[obstacles_id];
    return obstacles_context->obstacles;
}

bool obstacles_add(const obstacles_t obstacles_id, const obstacle_t obstacle)
{
    assert(obstacles_id < OBSTACLES_NUMOF);
    obstacles_context_t *obstacles_context = &obstacles_contexts[obstacles_id];
    if (obstacles_context->nb_obstacles + 1 >= OBSTACLES_MAX_NUMBER) {
        return false;
    }
    obstacles_context->obstacles[obstacles_context->nb_obstacles++] = obstacle;
    return true;
}

void obstacles_lock(const obstacles_t obstacles_id)
{
    assert(obstacles_id < OBSTACLES_NUMOF);
    obstacles_context_t *obstacles_context = &obstacles_contexts[obstacles_id];
    mutex_lock(&obstacles_context->lock);
}

void obstacles_unlock(const obstacles_t obstacles_id)
{
    assert(obstacles_id < OBSTACLES_NUMOF);
    obstacles_context_t *obstacles_context = &obstacles_contexts[obstacles_id];
    mutex_unlock(&obstacles_context->lock);
}

bool obstacles_is_point_in_obstacles(const coords_t *p, const obstacle_t *filter)
{
    for (size_t obstacles_id = 0; obstacles_id < OBSTACLES_NUMOF; obstacles_id++) {
        for (size_t j = 0; j < obstacles_get_nb_obstacles(obstacles_id); j++) {
            const obstacle_t *obstacle = obstacles_get(obstacles_id, j);
            if (filter == obstacle) {
                continue;
            }
            if (obstacles_is_point_in_obstacle(obstacle, p)) {
                return true;
            }
        }
    }

    return false;
}

bool obstacles_is_point_in_obstacle(const obstacle_t *obstacle, const coords_t *p)
{
    /* Temporary object */
    polygon_t polygon_tmp;

    switch (obstacle->type) {
        case OBSTACLE_CIRCLE:
            return collisions_is_point_in_circle(&obstacle->form.circle, p);
        case OBSTACLE_RECTANGLE:
            polygon_tmp.count = 4;
            memcpy(polygon_tmp.points, obstacle->form.rectangle.points,
                   polygon_tmp.count * sizeof(coords_t));
            return collisions_is_point_in_polygon(&polygon_tmp, p);
        case OBSTACLE_POLYGON:
            return collisions_is_point_in_polygon(&obstacle->form.polygon, p);
        default:
            DEBUG("obstacle: Wrong type, should never happen, return false)");
    }

    return false;
}

bool obstacles_is_segment_crossing_obstacle(const coords_t *a, const coords_t *b, const obstacle_t *obstacle)
{
    /* Temporary object */
    polygon_t polygon_tmp;

    switch (obstacle->type) {
        case OBSTACLE_CIRCLE:
            return collisions_is_segment_crossing_circle(a, b, &obstacle->form.circle);
        case OBSTACLE_RECTANGLE:
            polygon_tmp.count = 4;
            memcpy(polygon_tmp.points, obstacle->form.rectangle.points,
                   polygon_tmp.count * sizeof(coords_t));
            return collisions_is_segment_crossing_polygon(a, b, &polygon_tmp);
        case OBSTACLE_POLYGON:
            return collisions_is_segment_crossing_polygon(a, b, &obstacle->form.polygon);
        default:
            DEBUG("obstacle: Wrong type, should never happen, return false)");
    }

    return false;
}

coords_t obstacles_find_nearest_point_in_obstacle(const obstacle_t *obstacle,
                                                  const coords_t *p)
{
    /* Temporary object */
    polygon_t polygon_tmp;

    switch (obstacle->type) {
        case OBSTACLE_CIRCLE:
            return collisions_find_nearest_point_in_circle(&obstacle->form.circle, p);
        case OBSTACLE_RECTANGLE:
            polygon_tmp.count = 4;
            memcpy(polygon_tmp.points, obstacle->form.rectangle.points,
                   polygon_tmp.count * sizeof(coords_t));
            return collisions_find_nearest_point_in_polygon(&polygon_tmp, p);
        case OBSTACLE_POLYGON:
            return collisions_find_nearest_point_in_polygon(&obstacle->form.polygon, p);
        default:
            DEBUG("obstacle: Wrong type, should never happen, return false)");
    }

    /* Should never happen */
    return *p;
}

static void _print_circle(const obstacle_t *obstacle, tracefd_t out)
{
    tracefd_printf(
        out,
        "{\"x\":%.3lf,\"y\":%.3lf,\"radius\":%.3lf}",
        obstacle->center.x,
        obstacle->center.y,
        obstacle->form.circle.radius
        );
}

static void _print_rectangle(const obstacle_t *obstacle, tracefd_t out)
{
    tracefd_printf(
        out,
        "{\"x\":%.3lf,\"y\":%.3lf,\"angle\":%.3lf,\"length_x\":%.3lf,\"length_y\":%.3lf}",
        obstacle->center.x,
        obstacle->center.y,
        obstacle->angle,
        obstacle->form.rectangle.length_x,
        obstacle->form.rectangle.length_y
        );
}

static void _print_polygon(const obstacle_t *obstacle, tracefd_t out)
{
    const polygon_t *polygon = &obstacle->form.polygon;

    tracefd_printf(
        out,
        "{\"x\":%.3lf,\"y\":%.3lf,\"angle\":%.3lf,\"points\":[",
        obstacle->center.x,
        obstacle->center.y,
        obstacle->angle
        );
    for (uint8_t i = 0; i < polygon->count; i++) {
        if (i > 0) {
            tracefd_printf(out, ",");
        }
        tracefd_printf(
            out,
            "{\"x\":%.3lf,\"y\":%.3lf}",
            polygon->points[i].x,
            polygon->points[i].y
            );
    }
    tracefd_printf(out, "]}");
}

static void _print_list(const obstacles_t obstacles_id, tracefd_t out)
{
    assert(obstacles_id < OBSTACLES_NUMOF);
    obstacles_context_t *obstacles_context = &obstacles_contexts[obstacles_id];

    for (uint8_t i = 0; i < obstacles_context->nb_obstacles; i++) {
        if (i > 0) {
            tracefd_printf(out, ",");
        }

        obstacle_t *obstacle = &obstacles_context->obstacles[i];

        switch (obstacle->type) {
            case OBSTACLE_CIRCLE:
                _print_circle(obstacle, out);
                break;
            case OBSTACLE_RECTANGLE:
                _print_rectangle(obstacle, out);
                break;
            case OBSTACLE_POLYGON:
                _print_polygon(obstacle, out);
                break;
            default:
                DEBUG("obstacle: Wrong type, should never happen, return false)");
        }
    }
}

void obstacles_print_json(const obstacles_t obstacles_id, tracefd_t out)
{
    assert(obstacles_id < OBSTACLES_NUMOF);

    tracefd_printf(out, "[");

    _print_list(obstacles_id, out);

    tracefd_printf(out, "]");
}

void obstacles_print_all_json(tracefd_t out)
{
    size_t nb_obstacles = 0;

    tracefd_printf(out, "[");

    for (obstacles_t obstacles_id = 0; obstacles_id < OBSTACLES_NUMOF; obstacles_id++) {
        if (nb_obstacles > 0 && obstacles_get_nb_obstacles(obstacles_id) > 0) {
            tracefd_printf(out, ",");
        }

        _print_list(obstacles_id, out);
        nb_obstacles += obstacles_get_nb_obstacles(obstacles_id);
    }
    tracefd_printf(out, "]");
}
