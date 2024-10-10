#include <math.h>
#include <stdlib.h>

#include "odometer/OdometerDifferential.hpp"
#include "trigonometry.h"
#include "utils.hpp"

namespace cogip {

namespace odometer {

int OdometerDifferential::update()
{
    if (!left_encoder_ && !right_encoder_) {
        return -1;
    }
    
    /// Compute encoders wheels left and right linear delta in mm
    const double dL = left_encoder_->get_angle_and_reset() * parameters_.left_wheel_diameter_mm() * parameters_.left_polarity();
    const double dR = right_encoder_->get_angle_and_reset() * parameters_.right_wheel_diameter_mm() * parameters_.right_polarity();
    
    /// Compute linear delta for robot in mm
    const double delta_lin_pos      = (dL + dR) / 2;
    
    /// Compute angular delta for robot in rad
    const double delta_ang_pos      = (dR - dL) / parameters_.axle_track_mm();
    
    /// Compute angle in rad between -pi and pi
    pos_.O = limit_angle_rad(pos_.O + delta_ang_pos);
    
    /// Compute x and y coordinates in mm
    pos_.x += delta_lin_pos * cos(pos_.O);
    pos_.y += delta_lin_pos * sin(pos_.O); 
    
    /// velocites are in mm/period and rad/period so the current velocites are the deltas since last call
    vel_.linear  = delta_lin_pos;
    vel_.angular = delta_ang_pos;

    return 0;
}

} /// namespace odometer

} /// namespace cogip