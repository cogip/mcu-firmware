#include <stdio.h>
#include "avoidance.h"
#include "platform.h"
#include "trigonometry.h"
#include "obstacle.h"
#include <math.h>

/* Init borders */
void borders_init(polygon_t *polygon)
{
	polygon->count = 0;
	polygon->points[polygon->count++] = (pose_t){.x = AVOIDANCE_BORDER_X_MIN, .y = AVOIDANCE_BORDER_Y_MIN};
	polygon->points[polygon->count++] = (pose_t){.x = AVOIDANCE_BORDER_X_MAX, .y = AVOIDANCE_BORDER_Y_MIN};
	polygon->points[polygon->count++] = (pose_t){.x = AVOIDANCE_BORDER_X_MAX, .y = AVOIDANCE_BORDER_Y_MAX};
	polygon->points[polygon->count++] = (pose_t){.x = AVOIDANCE_BORDER_X_MIN, .y = AVOIDANCE_BORDER_Y_MAX};
}

/* Init onstacle borders */
static void obstacle_borders_init(polygon_t *polygon)
{
	polygon->count = 0;
	polygon->points[polygon->count++] = (pose_t){.x = OBSTACLE_BORDER_X_MIN, .y = OBSTACLE_BORDER_Y_MIN};
	polygon->points[polygon->count++] = (pose_t){.x = OBSTACLE_BORDER_X_MAX, .y = OBSTACLE_BORDER_Y_MIN};
	polygon->points[polygon->count++] = (pose_t){.x = OBSTACLE_BORDER_X_MAX, .y = OBSTACLE_BORDER_Y_MAX};
	polygon->points[polygon->count++] = (pose_t){.x = OBSTACLE_BORDER_X_MIN, .y = OBSTACLE_BORDER_Y_MAX};
}

/* Init all known fixed obstacles on map */
void mach_fixed_obstacles_init(void)
{
	polygon_t polygon;
	uint8_t nb_vertices;

	polygon.count = 0;
	nb_vertices = 4;
	if (nb_vertices < POLY_MAX_POINTS)
	{
		polygon.points[polygon.count++] = (pose_t){.x = -800, .y = 1550};
		polygon.points[polygon.count++] = (pose_t){.x =  800, .y = 1550};
		polygon.points[polygon.count++] = (pose_t){.x =  800, .y = 2200};
		polygon.points[polygon.count++] = (pose_t){.x = -800, .y = 2200};
		add_polygon(&polygon);
	}
}

/* Add a dynamic obstacle */
int8_t add_dyn_obstacle(const pose_t *robot_pose, sensor_t *sensor, double dist)
{
	static polygon_t obstacle_borders;
	polygon_t polygon;
	uint8_t nb_vertices;

	if (!obstacle_borders.count)
		obstacle_borders_init(&obstacle_borders);

	pose_t robot_pose_tmp = *robot_pose;
	robot_pose_tmp.O /= PULSE_PER_DEGREE;
	robot_pose_tmp.O += sensor->angle_robot_offset;
	robot_pose_tmp.O = (int16_t)robot_pose_tmp.O % 360;
	double angle = DEG2RAD(robot_pose_tmp.O);
	robot_pose_tmp.x /= PULSE_PER_MM;
	robot_pose_tmp.y /= PULSE_PER_MM;

	dist += sensor->dist_robot_offset_cm * 10;

	pose_t obstacle_point = (pose_t){.x = robot_pose_tmp.x + dist * cos(angle),
                             .y = robot_pose_tmp.y + dist * sin(angle),
                             .O = angle};

	if (!is_point_in_polygon(&obstacle_borders, obstacle_point)) {
		goto add_dyn_obstacle_error_obstacle_borders;
	}

	polygon.count = 0;
	nb_vertices = 4;
	if (nb_vertices < POLY_MAX_POINTS)
	{
		/* Right reference*/
		pose_t ref_pos_right = (pose_t){.x = robot_pose_tmp.x + OBSTACLE_DYN_SIZE/2 * cos(angle - M_PI/2),
							 .y = robot_pose_tmp.y + OBSTACLE_DYN_SIZE/2 * sin(angle - M_PI/2),
							 .O = angle /* + direction * M_PI/2 */};
		/* Left reference */
		pose_t ref_pos_left = (pose_t){.x = robot_pose_tmp.x + OBSTACLE_DYN_SIZE/2 * cos(angle + M_PI/2),
							 .y = robot_pose_tmp.y + OBSTACLE_DYN_SIZE/2 * sin(angle + M_PI/2),
							 .O = angle /* + direction * M_PI/2 */};

		/* Right points */
		/*polygon.points[polygon.count] = (pose_t){.x = ref_pos_right.x + dist * cos(ref_pos_right.O),
							   .y = ref_pos_right.y + dist * sin(ref_pos_right.O)};*/
		polygon.points[polygon.count] = ref_pos_right;
		polygon.count++;
		polygon.points[polygon.count] = (pose_t){.x = ref_pos_right.x + (dist + OBSTACLE_DYN_SIZE) * cos(ref_pos_right.O),
							 .y = ref_pos_right.y + (dist + OBSTACLE_DYN_SIZE) * sin(ref_pos_right.O)};
		polygon.count++;

		/* Left points */
		polygon.points[polygon.count] = (pose_t){.x = ref_pos_left.x + (dist + OBSTACLE_DYN_SIZE) * cos(ref_pos_left.O),
							 .y = ref_pos_left.y + (dist + OBSTACLE_DYN_SIZE) * sin(ref_pos_left.O)};
		polygon.count++;
		/*polygon.points[polygon.count] = (pose_t){.x = ref_pos_left.x + dist * cos(ref_pos_left.O),
							 .y = ref_pos_left.y + dist * sin(ref_pos_left.O)};*/
		polygon.points[polygon.count] = ref_pos_left;
		polygon.count++;
		//cons_printf("@o@,%+.0lf,%+.0lf,%+.0lf,%+.0lf,%+.0lf,%+.0lf,%+.0lf,%+.0lf,%+.0lf\n", polygon.points[0].x, polygon.points[0].y, polygon.points[1].x, polygon.points[1].y, polygon.points[2].x, polygon.points[2].y, polygon.points[3].x, polygon.points[3].y, robot_pose_tmp.O);
		//cons_printf("@t@,%+.0lf,%+.0lf,%+.0lf,%+.0lf\n", ref_pos_right.x, ref_pos_right.y, ref_pos_left.x, ref_pos_left.y);
		add_dyn_polygon(&polygon);
	}
	else
		goto add_dyn_obstacle_error_nb_vertices;

	return 0;

add_dyn_obstacle_error_nb_vertices:
add_dyn_obstacle_error_obstacle_borders:
	return -1;
}
