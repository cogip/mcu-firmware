#include <stdio.h>

#include "path.h"

#include "utils.h"

// TODO: do not use Rear gear.
static path_pose_t poses[] = {
	{ .pos = {.x =  873, .y =  163, .O =   45, }, .act = NULL, }, /* POSE_INITIAL */
	{ .pos = {.x = 1400, .y =  500, .O =  135, }, .act = NULL, },
	{ .pos = {.x = 1000, .y =  900, .O =  -90, }, .act = NULL, },
	{ .pos = {.x =  950, .y =  400, .O =  -90, }, .act = NULL, },
};

path_t robot_path = {
	/* static cfg */
	.play_in_loop = FALSE,
	.nb_pose = 4,
	.poses = poses,
};

