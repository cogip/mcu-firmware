#include <math.h>
#include <stdlib.h>

#include "odometry.hpp"
#include "trigonometry.h"
#include "utils.hpp"

/**
 * \fn void odometry_by_segment (const float distance, const float angle)
 * \brief update new robot pose (x, y, O) approximated by straight line
 *		segments
 * \param distance : delta value for distance [pulse]
 * \param angle : delta value for angle [pulse]
 */
static void
odometry_by_segment(cogip::cogip_defs::Pose &p, const float distance, const float angle)
{
    float O_rad;

    O_rad = DEG2RAD(p.O());

    p.set_x(p.x() + distance * cos(O_rad));
    p.set_y(p.y() + distance * sin(O_rad));
    p.set_O(limit_angle_deg(p.O() + angle));
}

/**
 * \fn void odometry_by_arc (const float distance, const float angle)
 * \brief update new robot pose (x, y, O) approximated by an arc
 * \param distance : delta value for distance [pulse]
 * \param angle : delta value for angle [pulse]
 */
static void
odometry_by_arc(cogip::cogip_defs::Pose &p, const float distance, const float angle)
{
    float O_rad;

    O_rad = DEG2RAD(p.O());

    if (angle == 0.0) {
        /* robot pose */
        p.set_x(p.x() + distance * cos(O_rad));
        p.set_y(p.y() + distance * sin(O_rad));
    }
    else {
        /* radius and angle of the arc */
        float a = DEG2RAD(angle);
        float r = distance / a;

        /* coordinates of the center of the arc */
        float xo = p.x() - r * sin(O_rad);
        float yo = p.y() + r * cos(O_rad);

        /* robot pose */
        p.set_O(limit_angle_deg(p.O() + angle));
        p.set_x(xo + r * sin(DEG2RAD(p.O())));
        p.set_y(yo - r * cos(DEG2RAD(p.O())));
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
