#include <stdio.h>

#include "path.h"
#include "platform.h"

#include "utils.h"

// TODO: do not use Rear gear.
static path_pose_t poses[] = {
	{ .pos = {.x =	-1000, .y =  200, .O =   0, }, .allow_reverse = TRUE, .act = NULL, }, /* POSE_INITIAL */
	{ .pos = {.x = 500, .y =  200, .O =  90, }, .allow_reverse = TRUE, .act = NULL, },
	{ .pos = {.x = 500, .y =  500, .O =  90, }, .allow_reverse = TRUE, .act = NULL, },
	{ .pos = {.x = 500, .y =  -500, .O =  90, }, .allow_reverse = TRUE, .act = NULL, },
	{ .pos = {.x = 500, .y =  0, .O =  0, }, .allow_reverse = TRUE, .act = NULL, },
	{ .pos = {.x =	0, .y =  0, .O =   0, }, .allow_reverse = TRUE, .act = NULL, }, /* POSE_INITIAL */
};

path_t robot_path = {
	/* static cfg */
	.play_in_loop = FALSE,
	.nb_pose = 6,
	.poses = poses,
};

