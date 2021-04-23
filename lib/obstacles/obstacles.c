/* System includes */
#include <math.h>

/* RIOT includes */
#include "mutex.h"

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

        /* Right reference*/
        pose_t ref_pos_right = {
            .x = origin->x + (obstacles_context->params.default_width / 2) * cos(obstacle_angle - M_PI / 2),
            .y = origin->y + (obstacles_context->params.default_width / 2) * sin(obstacle_angle - M_PI / 2),
            .O = obstacle_angle
        };

        /* Left reference */
        pose_t ref_pos_left = {
            .x = origin->x + (obstacles_context->params.default_width / 2) * cos(obstacle_angle + M_PI / 2),
            .y = origin->y + (obstacles_context->params.default_width / 2) * sin(obstacle_angle + M_PI / 2),
            .O = obstacle_angle
        };

        obstacles_context->obstacles[obstacles_context->nb_obstacles] = (obstacle_t){
            .angle = obstacle_angle,
            .points = {
                {
                    .x = ref_pos_right.x + (distance - obstacles_context->params.default_width / 2) * cos(ref_pos_right.O),
                    .y = ref_pos_right.y + (distance - obstacles_context->params.default_width / 2) * sin(ref_pos_right.O)
                },
                {
                    .x = ref_pos_right.x + (distance + obstacles_context->params.default_width / 2) * cos(ref_pos_right.O),
                    .y = ref_pos_right.y + (distance + obstacles_context->params.default_width / 2) * sin(ref_pos_right.O)
                },
                {
                    .x = ref_pos_left.x + (distance + obstacles_context->params.default_width / 2) * cos(ref_pos_left.O),
                    .y = ref_pos_left.y + (distance + obstacles_context->params.default_width / 2) * sin(ref_pos_left.O)
                },
                {
                    .x = ref_pos_left.x + (distance - obstacles_context->params.default_width / 2) * cos(ref_pos_left.O),
                    .y = ref_pos_left.y + (distance - obstacles_context->params.default_width / 2) * sin(ref_pos_left.O)
                }
            }
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

        fprintf(out, "{ \"angle\": %lf, \"points\": [", obstacles_context->obstacles[i].angle);
        fflush(out);
        for (int8_t j = 0; j < 4; j++) {
            if (j > 0) {
                fprintf(out, ", ");
                fflush(out);
            }
            fprintf(
                out,
                "{\"x\": %lf, \"y\": %lf, \"O\": %lf}",
                obstacles_context->obstacles[i].points[j].x,
                obstacles_context->obstacles[i].points[j].y,
                obstacles_context->obstacles[i].points[j].O
                );
            fflush(out);
        }
        fprintf(out, "]}");
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
