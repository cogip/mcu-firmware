#ifndef PLANNER_H_
#define PLANNER_H_

#include "odometry.h"
#include "path.h"
#include "utils.h"


pose_t planner_get_path_pose_initial(void);
void planner_start_game(void);

void *task_planner(void *arg);

#if defined(MODULE_CALIBRATION)
void planner_enter_calibration(void);
#endif /* MODULE_CALIBRATION */

#endif /* PLANNER_H_ */
