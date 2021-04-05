#include <math.h>
#include <stdlib.h>

#include "odometry.h"
#include "trigonometry.h"
#include "utils.h"

static double wheels_distance; /*!< robot wheels distance [pulse] */

inline int pose_equal(const pose_t *p1, const pose_t *p2)
{
    if ((p1 != NULL) && (p2 != NULL)) {
        return p1 == p2
               || (p1->x == p2->x && p1->y == p2->y && p1->O == p2->O);
    }
    else {
        return 0;
    }
}

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

    p->x += distance * cos(O_rad);
    p->y += distance * sin(O_rad);
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
        p->x += distance * cos(O_rad);
        p->y += distance * sin(O_rad);
    }
    else {
        /* radius and angle oh the arc */
        double a = DEG2RAD(angle);
        double r = distance / a;

        /* coordinates of the center oh the arc */
        double xo = p->x - r * sin(O_rad);
        double yo = p->y + r * cos(O_rad);

        /* robot pose */
        p->O += angle;
        p->x = xo + r * sin(O_rad);
        p->y = yo - r * cos(O_rad);

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
