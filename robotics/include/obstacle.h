#ifndef OBSTACLE_H_
#define OBSTACLE_H_

#include "avoidance.h"
#include "platform.h"

void borders_init(polygon_t *polygon);
void mach_fixed_obstacles_init(void);
int8_t add_dyn_obstacle(const uint16_t dev,
                        const pose_t *robot_pose,
                        double angle_offset,
                        double distance_offset,
                        double dist);

static const polygon_t obstacle_borders = {
    .points = {
        {
            .x = OBSTACLE_BORDER_X_MIN,
            .y = OBSTACLE_BORDER_Y_MIN
        },
        {
            .x = OBSTACLE_BORDER_X_MAX,
            .y = OBSTACLE_BORDER_Y_MIN
        },
        {
            .x = OBSTACLE_BORDER_X_MAX,
            .y = OBSTACLE_BORDER_Y_MAX
        },
        {
            .x = OBSTACLE_BORDER_X_MIN,
            .y = OBSTACLE_BORDER_Y_MAX
        },
    },
    .count = 4,
};

#endif /* OBSTACLE_H_ */
