#ifndef PLANNER_H_
#define PLANNER_H_

#include "ctrl.h"
#include "odometry.h"
#include "path.h"
#include "utils.h"

extern path_t *path;

void planner_start(ctrl_t*);

void *task_planner(void *arg);

#endif /* PLANNER_H_ */
