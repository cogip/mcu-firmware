#include <math.h>
#include <stdlib.h>

#include "Odometry.hpp"
#include "trigonometry.h"
#include "utils.hpp"

namespace cogip {

namespace odometry {

int Odometry::update(cogip::cogip_defs::Pose &current_pos, cogip::cogip_defs::Polar &current_speed)
{
    if (!left_encoder_ && !right_encoder_) {
        return -1;
    }

    const double dL = left_encoder_->get_traveled_distance();
    const double dR = right_encoder_->get_traveled_distance();

    const double delta_lin_pos      = (dL + dR) / 2;
    const double delta_ang_pos      = (dR - dL) / parameters_.get_axle_track();

    const double avg_theta = DEG2RAD(pos_.O()) + delta_ang_pos / 2;

    pos_.x += delta_lin_pos * cos(avg_theta);
    pos_.y += delta_lin_pos * sin(avg_theta);
    pos_.O = limit_angle_deg(pos_.O + RAD2DEG(delta_ang_pos)); 

    /* Compute delta value in second using time delta in order to get velocities in mm/s or rad/s */
    const float time_delta_sec = ((float)time_delta) / 1000;

    _vel.linear  = delta_lin_pos / time_delta_sec;
    _vel.angular = delta_ang_pos / time_delta_sec;
    
    /* Set outputs */
    current_pos.set_coords(cogip::cogip_defs::Coords(pos_.x, pos_.y));
    current_pos.set_O(pos_.O);
    
    current_speed.set_distance(_vel.linear);
    current_speed.set_angle(_vel.angular);

    return 0
}

} // namespace odometry

} // namespace cogip