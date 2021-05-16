/* System includes */
#include <math.h>

/* RIOT includes */
#include "mutex.h"

/* Project includes */
#include "collisions.h"
#include "obstacles.h"

#define ENABLE_DEBUG (0)
#include "debug.h"

/* Trigo */
#define M_PI                  3.14159265358979323846
#define DEG2RAD(a)            (a * (2.0 * M_PI) / 360.0)

/* Physical properties */
#define MAX_OBSTACLES         360


/**
 * @brief   Obstacles context
 */
typedef struct {
    obstacles_params_t params;                  /**< parameters */
    size_t nb_obstacles;
    obstacle_t obstacles[MAX_OBSTACLES];
    mutex_t lock;
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

double obstacles_default_radius(const obstacles_t obstacles_id)
{
    assert(obstacles_id < OBSTACLES_NUMOF);
    const obstacles_context_t *obstacles_context = &obstacles_contexts[obstacles_id];
    return obstacles_context->params.default_radius;
}

size_t obstacles_size(const obstacles_t obstacles_id)
{
    assert(obstacles_id < OBSTACLES_NUMOF);
    const obstacles_context_t *obstacles_context = &obstacles_contexts[obstacles_id];
    return obstacles_context->nb_obstacles;
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
    if (obstacles_context->nb_obstacles + 1 >= MAX_OBSTACLES) {
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

bool obstacles_is_point_in_obstacles(const pose_t *p, const obstacle_t *filter)
{
    for (size_t obstacles_id = 0; obstacles_id < OBSTACLES_NUMOF; obstacles_id++) {
        for (size_t j = 0; j < obstacles_size(obstacles_id); j++) {
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

bool obstacles_is_point_in_obstacle(const obstacle_t *obstacle, const pose_t *p)
{
    switch (obstacle->type) {
        case OBSTACLE_POLYGON:
            return collisions_is_point_in_polygon(&obstacle->form.polygon, p);
        case OBSTACLE_CIRCLE:
            return collisions_is_point_in_circle(&obstacle->form.circle, p);
        default:
            DEBUG("obstacle: Wrong type, should never happen, return false)");
    }

    return false;
}

pose_t obstacles_find_nearest_point_in_obstacle(const obstacle_t *obstacle,
                                                const pose_t *p)
{
    switch (obstacle->type) {
        case OBSTACLE_POLYGON:
            return collisions_find_nearest_point_in_polygon(&obstacle->form.polygon, p);
        case OBSTACLE_CIRCLE:
            return collisions_find_nearest_point_in_circle(&obstacle->form.circle, p);
        default:
            DEBUG("obstacle: Wrong type, should never happen, return false)");
    }

    /* Should never happen */
    return *p;
}

void obstacles_update_from_lidar(const obstacles_t obstacles_id, const pose_t *origin, const uint16_t *distances)
{
    assert(obstacles_id < OBSTACLES_NUMOF);
    obstacles_context_t *obstacles_context = &obstacles_contexts[obstacles_id];

    if (origin == NULL) {
        return;
    }

    obstacles_context->nb_obstacles = 0;

    for (uint16_t angle = 0; angle < 360; angle++) {
        uint16_t distance = distances[angle];
        if (distance < obstacles_context->params.min_distance || distance >= obstacles_context->params.max_distance) {
            continue;
        }

        /* Compute obstacle position */
        double obstacle_angle = origin->O + angle;
        obstacle_angle = (int16_t)obstacle_angle % 360;
        obstacle_angle = DEG2RAD(obstacle_angle);

        obstacles_context->obstacles[obstacles_context->nb_obstacles] = (obstacle_t){
            .type = OBSTACLE_CIRCLE,
            .form.circle.center = {
                .coords.x = origin->coords.x + distance * cos(obstacle_angle),
                .coords.y = origin->coords.y + distance * sin(obstacle_angle),
            },
            .form.circle.radius = obstacles_default_radius(obstacles_id),
        };

        obstacles_context->nb_obstacles++;
    }

}

static void _print_list(const obstacles_t obstacles_id, FILE *out)
{
    assert(obstacles_id < OBSTACLES_NUMOF);
    obstacles_context_t *obstacles_context = &obstacles_contexts[obstacles_id];

    for (uint8_t i = 0; i < obstacles_context->nb_obstacles; i++) {
        if (i > 0) {
            fprintf(out, ", ");
            fflush(out);
        }

        fprintf(
            out,
            "{\"x\":%lf,\"y\":%lf,\"radius\":%lf}",
            obstacles_context->obstacles[i].form.circle.center.coords.x,
            obstacles_context->obstacles[i].form.circle.center.coords.y,
            obstacles_context->obstacles[i].form.circle.radius
            );
        fflush(out);
    }
}

void obstacles_print_json(const obstacles_t obstacles_id, FILE *out)
{
    assert(obstacles_id < OBSTACLES_NUMOF);

    fprintf(out, "[");
    fflush(out);

    _print_list(obstacles_id, out);

    fprintf(out, "]\n");
    fflush(out);
}

void obstacles_print_all_json(FILE *out)
{
    size_t nb_obstacles = 0;

    fprintf(out, "[");
    fflush(out);

    for (obstacles_t obstacles_id = 0; obstacles_id < OBSTACLES_NUMOF; obstacles_id++) {
        if (nb_obstacles > 0 && obstacles_size(obstacles_id) > 0) {
            fprintf(out, ", ");
            fflush(out);
        }

        _print_list(obstacles_id, out);
        nb_obstacles += obstacles_size(obstacles_id);
    }
    fprintf(out, "]\n");
    fflush(out);
}
