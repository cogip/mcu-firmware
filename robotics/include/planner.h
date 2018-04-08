#ifndef PLANNER_H_
#define PLANNER_H_

#include "odometry.h"
#include "path.h"
#include "utils.h"

extern path_t * path;
extern uint8_t in_calibration;

void planner_start_game(void);
void increment_current_pose_idx(void);

void *task_planner(void *arg);

#if defined(MODULE_CALIBRATION)
void planner_enter_calibration(void);
#endif /* MODULE_CALIBRATION */

#endif /* PLANNER_H_ */
