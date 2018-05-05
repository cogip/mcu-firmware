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

/* Init all known fixed obstacles on map */
void mach_fixed_obstacles_init(void)
{
	polygon_t polygon;
	uint8_t nb_vertices;

	polygon.count = 0;
	nb_vertices = 4;
	if (nb_vertices < POLY_MAX_POINTS)
	{
		polygon.points[polygon.count++] = (pose_t){.x = -200, .y = 600};
		polygon.points[polygon.count++] = (pose_t){.x =  200, .y = 600};
		polygon.points[polygon.count++] = (pose_t){.x =  200, .y = 1000};
		polygon.points[polygon.count++] = (pose_t){.x = -200, .y = 1000};
		add_polygon(&polygon);
	}
}

/* Add a dynamic obstacle */
void add_dyn_obstacle(const pose_t *robot_pose/*, analog_sensor_zone_t zone*/)
{
	static polygon_t polygon;
	uint8_t nb_vertices;
	//double direction = -1;

	pose_t robot_pose_tmp = *robot_pose;
	robot_pose_tmp.O /= PULSE_PER_DEGREE;
	robot_pose_tmp.O = DEG2RAD(robot_pose_tmp.O);
	robot_pose_tmp.x /= PULSE_PER_MM;
	robot_pose_tmp.y /= PULSE_PER_MM;

	polygon.count = 0;
	nb_vertices = 4;
	if (nb_vertices < POLY_MAX_POINTS)
	{

		/*if (zone == AS_ZONE_REAR)
		{
			direction = 1;
		}*/

		/* Right reference*/
		pose_t ref_pos_right = (pose_t){.x = robot_pose_tmp.x + OBSTACLE_DYN_SIZE * cos(robot_pose_tmp.O - M_PI/2),
							 .y = robot_pose_tmp.y + OBSTACLE_DYN_SIZE * sin(robot_pose_tmp.O - M_PI/2),
							 .O = robot_pose_tmp.O /* + direction * M_PI/2 */};
		/* Left reference */
		pose_t ref_pos_left = (pose_t){.x = robot_pose_tmp.x + OBSTACLE_DYN_SIZE * cos(robot_pose_tmp.O + M_PI/2),
							 .y = robot_pose_tmp.y + OBSTACLE_DYN_SIZE * sin(robot_pose_tmp.O + M_PI/2),
							 .O = robot_pose_tmp.O /* + direction * M_PI/2 */};

		/* Right points */
		polygon.points[polygon.count] = (pose_t){.x = ref_pos_right.x + OBSTACLE_DYN_LENGTH * cos(ref_pos_right.O),
							   .y = ref_pos_right.y + OBSTACLE_DYN_LENGTH * sin(ref_pos_right.O)};
		polygon.count++;
		polygon.points[polygon.count] = (pose_t){.x = ref_pos_right.x + (OBSTACLE_DYN_LENGTH + OBSTACLE_DYN_SIZE) * cos(ref_pos_right.O),
							 .y = ref_pos_right.y + (OBSTACLE_DYN_LENGTH + OBSTACLE_DYN_SIZE) * sin(ref_pos_right.O)};
		polygon.count++;

		/* Left points */
		polygon.points[polygon.count] = (pose_t){.x = ref_pos_left.x + (OBSTACLE_DYN_LENGTH + OBSTACLE_DYN_SIZE) * cos(ref_pos_left.O),
							 .y = ref_pos_left.y + (OBSTACLE_DYN_LENGTH + OBSTACLE_DYN_SIZE) * sin(ref_pos_left.O)};
		polygon.count++;
		polygon.points[polygon.count] = (pose_t){.x = ref_pos_left.x + OBSTACLE_DYN_LENGTH * cos(ref_pos_left.O),
							 .y = ref_pos_left.y + OBSTACLE_DYN_LENGTH * sin(ref_pos_left.O)};
		polygon.count++;
		cons_printf("@o@,%+.0f,%+.0f,%+.0f,%+.0f,%+.0f,%+.0f,%+.0f,%+.0f,%+.0f\n", polygon.points[0].x, polygon.points[0].y, polygon.points[1].x, polygon.points[1].y, polygon.points[2].x, polygon.points[2].y, polygon.points[3].x, polygon.points[3].y, robot_pose->O/PULSE_PER_DEGREE);
		cons_printf("@t@,%+.0f,%+.0f,%+.0f,%+.0f\n", ref_pos_right.x, ref_pos_right.y, ref_pos_left.x, ref_pos_left.y);
		add_dyn_polygon(&polygon);
	}
}
