#ifndef PATH_H_
#define PATH_H_

#include "action.h"
#include "odometry.h"

typedef struct {
	pose_t pos;
	uint8_t allow_reverse;
	func_cb_t act;
} path_pose_t;

typedef struct {
	/* static cfg */
	uint8_t play_in_loop; /* for unit tests */
	uint8_t nb_pose;
	path_pose_t *poses;

	/* dynamic variables */
	uint8_t current_pose_idx;
} path_t;


extern path_t robot_path;

#endif /* PATH_H_ */
