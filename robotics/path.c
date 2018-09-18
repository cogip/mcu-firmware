#include <stdio.h>

#include "path.h"
#include "platform.h"

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
	/*{	.pos = {.x = 0, .y = 200, .O = 0, },
		.allow_reverse = FALSE,
		.max_speed = MAX_SPEED,
		.act = NULL, },
	{	.pos = {.x = 0, .y = 200, .O = 180, },
		.allow_reverse = FALSE,
		.max_speed = MAX_SPEED,
		.act = NULL, },*/

	 /* ********************************************************* */
	 /* POSE_INITIAL */
	{	.pos = {.x = -1500 +160 +10, .y = 150 +30, .O = 90, },
		.allow_reverse = TRUE,
		.max_speed = MAX_SPEED,
		.act = NULL, },

	/* Debug position for the same color water distributor */
	/*{	.pos = {.x = -1450 +160 +10 +22, .y = 850, .O = 90, },
		.allow_reverse = TRUE,
		.max_speed = MAX_SPEED,
		.act = NULL, },*/

	/* Goes to same color water distributor and start catching action */
	{	.pos = {.x = -1500 +160 +10, .y = 840, .O = 90, },
		.allow_reverse = TRUE,
		.max_speed = 50,
		.act = act_catch_same_color_water, }, //callback : act_catch_same_color_water

	/* Launch water into water tower */
	{	.pos = {.x = -1500 +160 +10, .y = 840 +500, .O = 90, },
		.allow_reverse = TRUE,
		.max_speed = MAX_SPEED,
		.act = act_launch_same_color_water, }, //callback : act_launch_same_color_water

	/* Goes to position to deploy arm for the bee (temp to avoid cube) */
	{	.pos = {.x = -1500 +700, .y = 840 +500, .O = 90, },
		.allow_reverse = TRUE,
		.max_speed = MAX_SPEED,
		.act = NULL, }, //callback : bras abeille en position de poussée

	/* Goes to position to deploy arm for the bee */
	{	.pos = {.x = -1500 +160 +100, .y = 2000 -160 -40, .O = 90, },
		.allow_reverse = TRUE,
		.max_speed = MAX_SPEED,
		.act = act_open_bee_pusher, }, //callback : bras abeille en position de poussée

	/* Goes to position to push the bee */
	{	.pos = {.x = -1500 +160 +100 +150, .y = 2000 -160 -40, .O = 0, },
		.allow_reverse = FALSE,
		.max_speed = MAX_SPEED,
		.act = act_open_bee_pusher, }, //callback : bras abeille en position de poussée

	/* Goes to position to close arm for the bee */
	{	.pos = {.x = -1500 +160 +100 +150, .y = 2000 -160 -40, .O = 0, },
		.allow_reverse = TRUE,
		.max_speed = MAX_SPEED,
		.act = act_close_bee_pusher, }, //callback : bras abeille en position initiale

	/* Goes to position to deploy arm for the bee (temp to avoid cube) */
	{	.pos = {.x = -1500 +300, .y = 1200, .O = 0, },
		.allow_reverse = TRUE,
		.max_speed = MAX_SPEED,
		.act = NULL, }, //callback : bras abeille en position de poussée

	/* Intermediate point to domotic pannel interruptor */
	{	.pos = {.x = -370, .y = 1000, .O = 90, },
		.allow_reverse = TRUE,
		.max_speed = MAX_SPEED,
		.act = act_close_bee_pusher, },

	/* Goes to domotic pannel interruptor */
	{	.pos = {.x = -370, .y = 150 +1, .O = 90, },
		.allow_reverse = TRUE,
		.max_speed = MAX_SPEED,
		.act = act_close_bee_pusher, },

	/* Intermediate point to domotic pannel interruptor */
	{	.pos = {.x = -370, .y = 300, .O = 90, },
		.allow_reverse = TRUE,
		.max_speed = MAX_SPEED,
		.act = act_close_bee_pusher, },

	/* Going to mixed color water distributor */
	{	.pos = {.x = 1250, .y = 2000 -160 -20, .O = 180, },
		.allow_reverse = TRUE,
		.max_speed = MAX_SPEED,
		.act = act_close_bee_pusher, },

	/* Goes to same color water distributor and start catching action */
	{	.pos = {.x = 890, .y = 2000 -160 -10, .O = 180, },
		.allow_reverse = FALSE,
		.max_speed = MAX_SPEED/2,
		.act = act_catch_same_color_water, }, //callback : act_catch_same_color_water

	/* Goes to position to launch mixed water into tower (temp to avoid cubes) */
	{	.pos = {.x = 0, .y = 1000, .O = 0, },
		.allow_reverse = TRUE,
		.max_speed = MAX_SPEED,
		.act = NULL, }, //callback : bras abeille en position de poussée

	/* Launch water into water tower */
	{	.pos = {.x = -1500 +160 +190, .y = 840 +500, .O = 85, },
		.allow_reverse = TRUE,
		.max_speed = MAX_SPEED,
		.act = act_launch_mixed_water, }, //callback : act_launch_same_color_water

	/* Launch water into water tower */
	{	.pos = {.x = -370, .y = 1540, .O = 90, },
		.allow_reverse = TRUE,
		.max_speed = MAX_SPEED,
		.act = act_drop_recycled_water, }, //callback : act_launch_same_color_water

	/* Goes to cube */
	{	.pos = {.x = -1500 +850, .y = 1500, .O = 90, },
		.allow_reverse = TRUE,
		.max_speed = MAX_SPEED,
		.act = act_close_bee_pusher, },

	/* Push cube */
	{	.pos = {.x = -1500 +850, .y = 360, .O = 90, },
		.allow_reverse = TRUE,
		.max_speed = MAX_SPEED,
		.act = act_close_bee_pusher, },

	/* Activate interruptor by rotation */
	/*{	.pos = {.x = -370, .y = 160 +10, .O = 180, },
		.allow_reverse = FALSE,
		.max_speed = MAX_SPEED,
		.act = NULL, },*/


	/*{ .pos = {.x = 500, .y = 500, .O = 0, }, .allow_reverse = TRUE, .act = NULL, },
	{ .pos = {.x = 500, .y = -500, .O = 0, }, .allow_reverse = TRUE, .act = NULL, },
	{ .pos = {.x = 500, .y = 0, .O = 0, }, .allow_reverse = TRUE, .act = NULL, },*/
};

path_t robot_path = {
	/* static cfg */
	.play_in_loop = FALSE,
	.nb_pose = 18, //10
	.poses = poses,
};

