#include <cmath>

#include "odometer/OdometerDifferential.hpp"
#include "trigonometry.h"
#include "utils.hpp"

namespace cogip {

namespace odometer {

int OdometerDifferential::update()
{
    // Compute encoders wheels left and right linear delta in mm
    const double dL = left_encoder_.get_angle_and_reset() * parameters_.left_wheel_diameter_mm() * parameters_.left_polarity();
    const double dR = right_encoder_.get_angle_and_reset() * parameters_.right_wheel_diameter_mm() * parameters_.right_polarity();

    // Compute linear delta for robot in mm
    const double delta_linear_pose      = (dL + dR) / 2;

    // Compute angular delta for robot in rad
    const double delta_angular_pose      = (dR - dL) / parameters_.track_width_mm();

    // Compute angle in rad between -pi and pi
    double O_rad = DEG2RAD(pose_.O());
    O_rad = limit_angle_rad(O_rad + delta_angular_pose);
    
    // Compute x and y coordinates in mm
    pose_.set_x(pose_.x() + delta_linear_pose * cos(O_rad)); 
    pose_.set_y(pose_.y() + delta_linear_pose * sin(O_rad));
    pose_.set_O(RAD2DEG(O_rad));
    
    // Save polar pose delta since last call
    polar_.set_distance(delta_linear_pose);
    polar_.set_angle(RAD2DEG(delta_angular_pose));

    return 0;
}

} // namespace odometer

} // namespace cogip