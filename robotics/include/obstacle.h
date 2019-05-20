#ifndef OBSTACLE_H_
#define OBSTACLE_H_

#include "avoidance.h"

#define OBSTACLE_DYN_LENGTH         300
#define OBSTACLE_DYN_SIZE           800
#define OBSTACLE_DETECTION_TRESHOLD 2000

void borders_init(polygon_t *polygon);
void mach_fixed_obstacles_init(void);
int8_t add_dyn_obstacle(const uint16_t dev,
                        const pose_t *robot_pose,
                        double angle_offset,
                        double distance_offset,
                        double dist);

#endif /* OBSTACLE_H_ */
