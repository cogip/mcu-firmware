#ifndef OBSTACLE_H_
#define OBSTACLE_H_

#define OBSTACLE_DYN_LENGTH 300
#define OBSTACLE_DYN_SIZE   400

void borders_init(polygon_t *polygon);
void mach_fixed_obstacles_init(void);
void add_dyn_obstacle(const pose_t *robot_pose/*, analog_sensor_zone_t zone*/);

#endif /* OBSTACLE_H_ */
