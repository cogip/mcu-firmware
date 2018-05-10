#ifndef OBSTACLE_H_
#define OBSTACLE_H_

#include "analog_sensor.h"

#define OBSTACLE_DYN_SIZE   900

void borders_init(polygon_t *polygon);
void mach_fixed_obstacles_init(void);
int8_t add_dyn_obstacle(const pose_t *robot_pose, sensor_t *sensor, double dist);

#endif /* OBSTACLE_H_ */
