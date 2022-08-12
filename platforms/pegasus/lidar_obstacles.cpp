#include "lidar_obstacles.hpp"

// System includes
#include <cmath>

// RIOT includes
#include <ztimer.h>

// Project includes
#include "lds01.h"

// Application includes
#include "lidar_utils.hpp"
#include "obstacles/Circle.hpp"
#include "platform.hpp"

// Number of angles without obstacle to ignore between two angles with obstacle
#define NB_ANGLES_WITHOUT_OBSACLE_TO_IGNORE 3

// Periodic task
#define TASK_PERIOD_USEC    (200 * US_PER_MS)

// Thread priority
#define OBSTACLE_UPDATER_PRIO (THREAD_PRIORITY_MAIN - 1)

// Trigo
#define M_PI                  3.14159265358979323846
#define DEG2RAD(a)            (a * (2.0 * M_PI) / 360.0)

// Thread stack
static char obstacle_updater_thread_stack[THREAD_STACKSIZE_LARGE * 2];

// Obstacles list of Lidar detected obstacles
cogip::obstacles::List *lidar_obstacles = nullptr;

// Find consecutive obstacles and keep the nearest at the middle
static bool _filter_distances(const uint16_t *raw_distances, uint16_t *filtered_distances)
{
    for (uint32_t i = 0; i < LDS01_NB_ANGLES; i++) {
        filtered_distances[i] = LIDAR_MAX_DISTANCE;
    }

    // Find an angle without obstacle
    int start = -1;
    for (uint32_t i = 0; i < LDS01_NB_ANGLES; i++) {
        if (raw_distances[i] >= LIDAR_MAX_DISTANCE) {
            start = i;
            break;
        }
    }

    // Exit if no obstacle deteted
    if (start == -1) {
        return false;
    }

    // Iterate over all angles, starting by the first angle without obstacle
    for (uint32_t first = start; first < start + LDS01_NB_ANGLES; first++) {
        uint32_t dist_min = raw_distances[first % LDS01_NB_ANGLES];

        // Find the next angle with obstacle
        if (dist_min >= LIDAR_MAX_DISTANCE) {
            continue;
        }

        // An angle (first) with obstacle is found, iterate until the next obstacle without obstacle
        for (uint32_t last = first + 1; last <= start + LDS01_NB_ANGLES; last++) {
            uint16_t dist_current = raw_distances[last % LDS01_NB_ANGLES];

            // Keep the nearest distance of consecutive obstacles
            if (dist_current < LIDAR_MAX_DISTANCE
                || raw_distances[(last + 1) % LDS01_NB_ANGLES] < LIDAR_MAX_DISTANCE) {
                dist_min = dist_current < dist_min ? dist_current : dist_min;
                continue;
            }

            // Don't exit the loop if only NB_ANGLES_WITHOUT_OBSACLE_TO_IGNORE consecutive angles have
            // no obstacle
            bool continue_loop = false;
            for (uint32_t next = last + 1; next < last + NB_ANGLES_WITHOUT_OBSACLE_TO_IGNORE; next++) {
                if (raw_distances[next % LDS01_NB_ANGLES] < LIDAR_MAX_DISTANCE) {
                    continue_loop = true;
                    break;
                }
            }
            if (continue_loop) {
                continue;
            }

            // Only keep one angle at the middle of the consecutive angles with obstacles
            // Set its distance to the minimun distance of the range
            uint32_t middle = first + (int)((float)(last - first) / 2 + 0.5);
            filtered_distances[middle % LDS01_NB_ANGLES] = dist_min;
            first = last + 1;
            break;
        }
    }

    return true;
}

// Update obstacles list from lidar measurements
static void _update_dynamic_obstacles_from_lidar(cogip::obstacles::List * obstacles,
                                                 const cogip::cogip_defs::Pose &origin,
                                                 const uint16_t *distances)
{
    obstacles->clear();

    for (uint16_t angle = 0; angle < 360; angle++) {
        uint16_t distance = distances[angle];
        if (distance < LIDAR_MIN_DISTANCE || distance >= LIDAR_MAX_DISTANCE) {
            continue;
        }

        // Compute obstacle position
        double obstacle_angle = origin.O() + angle;
        obstacle_angle = (int16_t)obstacle_angle % 360;
        obstacle_angle = DEG2RAD(obstacle_angle);

        cogip::cogip_defs::Coords center(
            origin.x() + distance * cos(obstacle_angle),
            origin.y() + distance * sin(obstacle_angle)
        );

        obstacles->push_back(new cogip::obstacles::Circle(center, LIDAR_OBSTACLE_RADIUS));
    }
}

// Thread loop
static void *_thread_obstacle_updater(void *arg)
{
    cogip::cogip_defs::Pose *robot_state = (cogip::cogip_defs::Pose *)arg;
    uint16_t raw_distances[LDS01_NB_ANGLES];
    uint16_t filtered_distances[LDS01_NB_ANGLES];

    // Init loop iteration start time
    ztimer_now_t loop_start_time = ztimer_now(ZTIMER_USEC);

    while (true) {
        lds01_get_distances(lidar_get_device(), raw_distances);
        // A Lidar distance is from the robot center to the edge of beacon support
        // Adjust the distance to point the center of the beacon support radius
        for (uint32_t i = 0; i < LDS01_NB_ANGLES; i++) {
            raw_distances[i] += BEACON_SUPPORT_DIAMETER / 2;
        }

        if (_filter_distances(raw_distances, filtered_distances) == true) {
            _update_dynamic_obstacles_from_lidar(lidar_obstacles, *robot_state, filtered_distances);
        }

        // Wait thread period to end
        ztimer_periodic_wakeup(ZTIMER_USEC, &loop_start_time, TASK_PERIOD_USEC);
    }

    return EXIT_SUCCESS;
}

void obstacle_updater_start(const cogip::cogip_defs::Pose &robot_state)
{
    lidar_obstacles = new cogip::obstacles::List();

    // Start the obstacle updater thread
    thread_create(
        obstacle_updater_thread_stack,
        sizeof(obstacle_updater_thread_stack),
        OBSTACLE_UPDATER_PRIO,
        0,
        _thread_obstacle_updater,
        (void *)&robot_state,
        "Obstacle updater"
        );
}
