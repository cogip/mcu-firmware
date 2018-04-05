#include "controller.h"
#include "planner.h"
#include "platform.h"

static void planner_calibration_usage(void)
{
	cons_printf("\n>>> Entering planner calibration mode\n\n");

	cons_printf("\t'n' to go to next pose in path\n");
	cons_printf("\t'p' to go to prev pose in path\n");
	cons_printf("\t'a' to launch current action at current pose\n");
	cons_printf("\n");
	cons_printf("\t'v' to dump all pose in path\n");
	cons_printf("\t'x' to tune next pose x\n");
	cons_printf("\t'y' to tune next pose y\n");
	cons_printf("\t'O' to tune next pose O\n");
	cons_printf("\t'h' to display this help\n");
	cons_printf("\t'q' to quit\n");
	cons_printf("\n");
}

static void scanf_update_val (const char *var_name, double *var)
{
	cons_printf("%s = %+.2f\tenter new value: ", var_name , *var);
	cons_scanf("%lf", var);
	cons_printf("new %s = %+.2f\n", var_name, *var);
}

void planner_enter_calibration(void)
{
	int c, i;
	uint8_t quit = 0;

	planner_calibration_usage();

	path = mach_get_path();
	if (!path) {
		printf("machine has no path\n");
		return;
	}

	in_calibration = TRUE;
	controller_set_mode(&controller, &controller_modes[CTRL_STATE_INGAME]);

	while (!quit) {

		/* display prompt */
		cons_printf("$ ");

		/* wait for command */
		c = cons_getchar();
		cons_printf("%c\n", c);

		switch (c) {
		case 'n':
			increment_current_pose_idx();
			break;
		case 'p':
			if (path->current_pose_idx)
				path->current_pose_idx--;
			break;
		case 'a':
			if (path->poses[path->current_pose_idx].act)
				path->poses[path->current_pose_idx].act();
			break;
		case 'x':
			if (path->current_pose_idx+1 < path->nb_pose)
				scanf_update_val("x", &path->poses[path->current_pose_idx+1].pos.x);
			break;
		case 'y':
			if (path->current_pose_idx+1 < path->nb_pose)
				scanf_update_val("y", &path->poses[path->current_pose_idx+1].pos.y);
			break;
		case 'O':
			if (path->current_pose_idx+1 < path->nb_pose)
				scanf_update_val("O", &path->poses[path->current_pose_idx+1].pos.O);
			break;
		case 'v':
			cons_printf("nb_pose = %d\n", path->nb_pose);
			for (i = 0; i < path->nb_pose; i++) {
				pose_t *p = &path->poses[i].pos;
				cons_printf("%02d %c\t"
					   "X = %+.2f\tY = %+.2f\tO = %+.2f\n",
					   i,
					   i == path->current_pose_idx ? '<' : ' ',
					   p->x, p->y, p->O);
			}
			break;
		case 'h':
			planner_calibration_usage();
			break;
		case 'q':
			quit = 1;
			break;
		default:
			cons_printf("\n");
			break;
		}
	}

	path->current_pose_idx = 0;
	controller_set_mode(&controller, &controller_modes[CTRL_STATE_IDLE]);
	in_calibration = FALSE;
}
