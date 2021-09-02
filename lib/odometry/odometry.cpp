#include <math.h>
#include <stdlib.h>

#include "odometry.hpp"
#include "trigonometry.h"
#include "utils.h"

static double wheels_distance; /*!< robot wheels distance [pulse] */

void odometry_setup(double d)
{
    wheels_distance = d;
}

/**
 * \fn void limit_angle (void)
 * \brief limit angle from -pi to +pi
 */
static void limit_angle(pose_t *p)
{
    /* limit angle from -pi to +pi */
    if (p->O > (M_PI * wheels_distance)) { /* > +pi */
        p->O -= 2.0 * (M_PI * wheels_distance);
    }

    if (p->O < -(M_PI * wheels_distance)) { /* < -pi */
        p->O += 2.0 * (M_PI * wheels_distance);
    }
}

/**
 * \fn void odometry_by_segment (const double distance, const double angle)
 * \brief update new robot pose_t (x, y, O) approximated by straight line
 *		segments
 * \param distance : delta value for distance [pulse]
 * \param angle : delta value for angle [pulse]
 */
static void
odometry_by_segment(pose_t *p, const double distance, const double angle)
{
    double O_rad;

    O_rad = DEG2RAD(p->O);

    p->coords.x += distance * cos(O_rad);
    p->coords.y += distance * sin(O_rad);
    p->O += angle;

    limit_angle(p);
}

/**
 * \fn void odometry_by_arc (const double distance, const double angle)
 * \brief update new robot pose_t (x, y, O) approximated by an arc
 * \param distance : delta value for distance [pulse]
 * \param angle : delta value for angle [pulse]
 */
static void
odometry_by_arc(pose_t *p, const double distance, const double angle)
{
    double O_rad;

    O_rad = DEG2RAD(p->O);

    if (angle == 0.0) {
        /* robot pose */
        p->coords.x += distance * cos(O_rad);
        p->coords.y += distance * sin(O_rad);
    }
    else {
        /* radius and angle oh the arc */
        double a = DEG2RAD(angle);
        double r = distance / a;

        /* coordinates of the center oh the arc */
        double xo = p->coords.x - r * sin(O_rad);
        double yo = p->coords.y + r * cos(O_rad);

        /* robot pose */
        p->O += angle;
        p->coords.x = xo + r * sin(O_rad);
        p->coords.y = yo - r * cos(O_rad);

        limit_angle(p);
    }
}

void
odometry_update(pose_t *p, polar_t *robot_speed, const uint8_t approximation)
{
    if (approximation == ARC) {
        odometry_by_arc(p, robot_speed->distance, robot_speed->angle);
    }
    else {
        odometry_by_segment(p,
                            robot_speed->distance, robot_speed->angle);
    }
}
