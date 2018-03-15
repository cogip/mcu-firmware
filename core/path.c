#include <stdio.h>

#include "path.h"

#include "utils.h"

// TODO: do not use Rear gear.
static path_pose_t poses_homologation_yellow[] = {
	{ .pos = {.x =  873, .y =  163, .O =   45, }, .act = NULL, }, /* POSE_INITIAL */
	{ .pos = {.x = 1400, .y =  500, .O =  135, }, .act = NULL, },
	{ .pos = {.x = 1000, .y =  900, .O =  -90, }, .act = NULL, },
	{ .pos = {.x =  950, .y =  400, .O =  -90, }, .act = NULL, },
};

path_t path_homologation_yellow = {
	/* static cfg */
	.play_in_loop = FALSE,
	.nb_pose = 4,
	.poses = poses_homologation_yellow,
};

static path_pose_t path_game_yellow[] = {
	{ .pos = {.x =  873, .y =  163, .O =   45, }, .act = NULL, }, /* POSE_INITIAL */
	{ .pos = {.x = 1117, .y =  235, .O =    0, }, .act = act_catch_module_front_right, },
	{ .pos = {.x = 1187, .y =  235, .O =    0, }, .act = act_catch_module_rear_right, },
	{ .pos = {.x = 1187, .y =  235,	.O =  180, }, .act = act_catch_module_front_left, },
	{ .pos = {.x = 1117, .y =  235, .O =  180, }, .act = act_catch_module_rear_left, },
	{ .pos = {.x = 1267, .y = 1150,	.O =   90, }, .act = NULL, },
};

path_t path_yellow = {
	/* static cfg */
	.play_in_loop = FALSE,
	.nb_pose = 6,
	.poses = path_game_yellow,
};
