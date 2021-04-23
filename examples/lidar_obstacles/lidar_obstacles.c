/* System includes */
#include <stdio.h>
#include <stdlib.h>

/* RIOT includes */
#include "xtimer.h"

/* Project includes */
#include "lds01.h"

/* Application includes */
#include "common_defs.h"
#include "lidar_utils.h"
#include "obstacles.h"

/* Number of angles without obstacle to ignore between two angles with obstacles */
#define NB_ANGLES_WITHOUT_OBSACLE_TO_IGNORE 3

/* Periodic task */
#define TASK_PERIOD_MS        50

/* Thread priority */
#define OBSTACLE_UPDATER_PRIO (THREAD_PRIORITY_MAIN - 1)

/* Thread stack */
static char obstacle_updater_thread_stack[THREAD_STACKSIZE_LARGE];

/* Obstacles id of Lidar detected obstacles*/
obstacles_t lidar_obstacles = OBSTACLES_NUMOF;

/* Lidar obstacles parameters */
obstacles_params_t lidar_obstacles_params = {
    .default_width = OBSTACLE_WIDTH * 2,
    .min_distance = ROBOT_WIDTH / 2 + 100,
    .max_distance = LIDAR_MAX_DISTANCE - OBSTACLE_WIDTH / 2 + BEACON_SUPPORT_DIAMETER / 2
};

/* Find consecutive obstacles and keep the nearest at the middle */
static bool _filter_distances(const uint16_t *raw_distances, uint16_t *filtered_distances)
{
    for (uint32_t i = 0; i < LDS01_NB_ANGLES; i++) {
        filtered_distances[i] = lidar_obstacles_params.max_distance;
    }

    /* Find an angle without obstacle */
    int start = -1;
    for (uint32_t i = 0; i < LDS01_NB_ANGLES; i++) {
        if (raw_distances[i] >= lidar_obstacles_params.max_distance) {
            start = i;
            break;
        }
    }

    /* Exit if no obstacle deteted */
    if (start == -1) {
        return false;
    }

    /* Iterate over all angles, starting by the first angle without obstacle */
    for (uint32_t first = start; first < start + LDS01_NB_ANGLES; first++) {
        uint32_t dist_min = raw_distances[first % LDS01_NB_ANGLES];

        /* Find the next angle with obstacle */
        if (dist_min >= lidar_obstacles_params.max_distance) {
            continue;
        }

        /* An angle (first) with obstacle is found, iterate until the next obstacle without obstacle */
        for (uint32_t last = first + 1; last <= start + LDS01_NB_ANGLES; last++) {
            uint16_t dist_current = raw_distances[last % LDS01_NB_ANGLES];

            /* Keep the nearest distance of consecutive obstacles */
            if (dist_current < lidar_obstacles_params.max_distance
                || raw_distances[(last + 1) % LDS01_NB_ANGLES] < lidar_obstacles_params.max_distance) {
                dist_min = dist_current < dist_min ? dist_current : dist_min;
                continue;
            }

            /* Don't exit the loop if only 3 consecutive angles have no obstacle */
            bool continue_loop = false;
            for (uint32_t next = last + 1; next < last + NB_ANGLES_WITHOUT_OBSACLE_TO_IGNORE; next++) {
                if (raw_distances[next % LDS01_NB_ANGLES] < lidar_obstacles_params.max_distance) {
                    continue_loop = true;
                    break;
                }
            }
            if (continue_loop) {
                continue;
            }

            /* Only keep one angle at the middle of the consecutive angles with obstacles */
            /* Set its distance to the minimun distance of the range */
            uint32_t middle = first + (int)((float)(last - first) / 2 + 0.5);
            filtered_distances[middle % LDS01_NB_ANGLES] = dist_min;
            first = last + 1;
            break;
        }
    }

    return true;
}

/* Thread loop */
static void *_thread_obstacle_updater(void *arg)
{
    const pose_t *robot_state = (pose_t *)arg;
    uint16_t raw_distances[LDS01_NB_ANGLES];
    uint16_t filtered_distances[LDS01_NB_ANGLES];

    for (;;) {
        xtimer_ticks32_t loop_start_time = xtimer_now();

        lds01_get_distances(lidar_get_device(), raw_distances);
        /* A Lidar distance is from the robot center to the edge of beacon support */
        /* Adjust the distance to point the center of the beacon support width */
        for (uint32_t i = 0; i < LDS01_NB_ANGLES; i++) {
            raw_distances[i] += BEACON_SUPPORT_DIAMETER / 2;
        }

        if (_filter_distances(raw_distances, filtered_distances) == true) {
            obstacles_update_from_lidar(lidar_obstacles, robot_state, filtered_distances);
        }

        xtimer_periodic_wakeup(&loop_start_time, TASK_PERIOD_MS * US_PER_MS);
    }

    return EXIT_SUCCESS;
}

void obstacle_updater_start(const pose_t *robot_state)
{
    lidar_obstacles = obstacles_init(&lidar_obstacles_params);

    /* Start the obstacle updater thread */
    thread_create(
        obstacle_updater_thread_stack,
        sizeof(obstacle_updater_thread_stack),
        OBSTACLE_UPDATER_PRIO,
        0,
        _thread_obstacle_updater,
        (void *)robot_state,
        "Obstacle updater"
        );
}
