#include <math.h>
#include <stdlib.h>

#include "odometry.hpp"
#include "trigonometry.h"
#include "utils.hpp"

static double wheels_distance; /*!< robot wheels distance [pulse] */

void odometry_setup(double d)
{
    wheels_distance = d;
}

/**
 * \fn void limit_angle (void)
 * \brief limit angle from -pi to +pi
 */
static void limit_angle(cogip::cogip_defs::Pose &p)
{
    /* limit angle from -pi to +pi */
    if (p.O() > (M_PI * wheels_distance)) { /* > +pi */
        p.set_O(p.O() - 2.0 * (M_PI * wheels_distance));
    }

    if (p.O() < -(M_PI * wheels_distance)) { /* < -pi */
        p.set_O(p.O() + 2.0 * (M_PI * wheels_distance));
    }
}

/**
 * \fn void odometry_by_segment (const double distance, const double angle)
 * \brief update new robot pose (x, y, O) approximated by straight line
 *		segments
 * \param distance : delta value for distance [pulse]
 * \param angle : delta value for angle [pulse]
 */
static void
odometry_by_segment(cogip::cogip_defs::Pose &p, const double distance, const double angle)
{
    double O_rad;

    O_rad = DEG2RAD(p.O());

    p.set_x(p.x() + distance * cos(O_rad));
    p.set_y(p.y() + distance * sin(O_rad));
    p.set_O(p.O() + angle);

    limit_angle(p);
}

/**
 * \fn void odometry_by_arc (const double distance, const double angle)
 * \brief update new robot pose (x, y, O) approximated by an arc
 * \param distance : delta value for distance [pulse]
 * \param angle : delta value for angle [pulse]
 */
static void
odometry_by_arc(cogip::cogip_defs::Pose &p, const double distance, const double angle)
{
    double O_rad;

    O_rad = DEG2RAD(p.O());

    if (angle == 0.0) {
        /* robot pose */
        p.set_x(p.x() + distance * cos(O_rad));
        p.set_y(p.y() + distance * sin(O_rad));
    }
    else {
        /* radius and angle of the arc */
        double a = DEG2RAD(angle);
        double r = distance / a;

        /* coordinates of the center of the arc */
        double xo = p.x() - r * sin(O_rad);
        double yo = p.y() + r * cos(O_rad);

        /* robot pose */
        p.set_O(p.O() + angle);
        p.set_x(xo + r * sin(O_rad));
        p.set_y(yo - r * cos(O_rad));

        limit_angle(p);
    }
}

void
odometry_update(
    cogip::cogip_defs::Pose &p,
    const cogip::cogip_defs::Polar &robot_speed,
    const uint8_t approximation)
{
    if (approximation == ARC) {
        odometry_by_arc(p, robot_speed.distance(), robot_speed.angle());
    }
    else {
        odometry_by_segment(p,
                            robot_speed.distance(), robot_speed.angle());
    }
}
