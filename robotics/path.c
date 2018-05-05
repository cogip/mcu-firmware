#include <stdio.h>

#include "path.h"

#include "utils.h"

/* NOTES
 *
 * Width Robot : 320 mm -> Middle at 160 mm
 * Depth Robot : 300 mm -> Middle at 150 mm
 *
 * Border width : 22 mm
 */


// TODO: do not use Rear gear.
static path_pose_t poses[] = {
	 /* POSE_INITIAL */
	{ .pos = {.x = -1500 +160 +10 +22, .y = 150, .O = 90, }, .allow_reverse = TRUE, .act = NULL, },

	/* Debug position for the same color water distributor */
	{ .pos = {.x = -1450 +160 +10 +22, .y = 850, .O = 90, }, .allow_reverse = TRUE, .act = NULL, },

	/* Goes to same color water distributor and start catching action */
	//{ .pos = {.x = -1500 +160 +10 +22, .y = 850, .O = 90, }, .allow_reverse = TRUE, .act = NULL, },
	//callback : act_catch_same_color_water

	/* Launch water into water tower */
	//{ .pos = {.x = -1500 +160 +10 +22, .y = 850, .O = 90, }, .allow_reverse = TRUE, .act = NULL, }, 
	//callback : act_launch_same_color_water

	/* Goes to position to push the bee */
	{ .pos = {.x = -1500 +160 +10 +22, .y = 2000 -150 -100, .O = 90, }, .allow_reverse = TRUE, .act = NULL, }, //callback : baisser bras gauche.

	/* Turn in order to push the bee */
	{ .pos = {.x = -1500 +160 +10 +22, .y = 2000 -150 -100, .O = 0, }, .allow_reverse = TRUE, .act = NULL, },

	/* Intermediate point to domotic pannel interruptor */
	{ .pos = {.x = -770, .y = 160 + 20, .O = 0, }, .allow_reverse = TRUE, .act = NULL, },

	/* Goes to domotic pannel interruptor */
	{ .pos = {.x = -370, .y = 160 + 20, .O = 0, }, .allow_reverse = TRUE, .act = NULL, },

	/* Activate interruptor by rotation */
	{ .pos = {.x = -370, .y = 160 + 20, .O = 180, }, .allow_reverse = TRUE, .act = NULL, },

	/* Going to mixed color water distributor */
	{ .pos = {.x = 1250, .y = 2000 -150 -50, .O = 180, }, .allow_reverse = TRUE, .act = NULL, },

	/*{ .pos = {.x = 500, .y = 500, .O = 0, }, .allow_reverse = TRUE, .act = NULL, },
	{ .pos = {.x = 500, .y = -500, .O = 0, }, .allow_reverse = TRUE, .act = NULL, },
	{ .pos = {.x = 500, .y = 0, .O = 0, }, .allow_reverse = TRUE, .act = NULL, },*/
};

path_t robot_path = {
	/* static cfg */
	.play_in_loop = FALSE,
	.nb_pose = 8, //9
	.poses = poses,
};

