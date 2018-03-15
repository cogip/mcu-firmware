#ifndef ODOMETRY_H_
#define ODOMETRY_H_

#include <stdint.h>

#define SEGMENT		0
#define ARC		1

/**
 * \struct pose
 */
typedef struct {
	double x; /*!< x-position [pulse] */
	double y; /*!< y-position [pulse] */
	double O; /*!< 0-orientation [pulse] */
} pose_t;

typedef struct {
	double distance;
	double angle;
} polar_t;

uint8_t pose_equal(const pose_t *p1, const pose_t *p2);

/**
 * \fn odometry_setup
 * \brief odometry pose_t and wheels_distance setup
 * \param p : robot pose
 * \param d : distance between wheels [pulse]
 */
void odometry_setup(double d);

/**
 * \fn odometry_update
 * \brief update new robot pose_t (x, y, O) by ARC or SEGMENT approximation
 * \param polar_t : delta value for distance [pulse]
 * \param approximation : SEGMENT (default) or ARC
 */
void odometry_update(pose_t *p,
		     polar_t *robot_speed,
		     const uint8_t approximation);

#endif /* ODOMETRY_H_ */
