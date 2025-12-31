// Project includes
#include "pid/PID.hpp"
#include "etl/algorithm.h"

namespace cogip {

namespace pid {

float PID::compute(float error)
{
    float p, i, d;

    // Anti-windup: conditional integration
    // Only integrate if:
    // - Error pushes integral toward zero (opposite signs), OR
    // - Integral hasn't reached the limit yet
    float limit = parameters_.integral_term_limit.get();
    bool error_reduces_integral = (error * integral_term_) <= 0;
    bool integral_below_limit = (integral_term_ < limit) && (integral_term_ > -limit);

    if (error_reduces_integral || integral_below_limit) {
        integral_term_ += error;

        // Clamp integral to limits
        integral_term_ = etl::min(integral_term_, limit);
        integral_term_ = etl::max(integral_term_, -limit);
    }

    // Proportional
    p = error * parameters_.kp.get();

    // Integral
    i = integral_term_ * parameters_.ki.get();

    // Derivative
    d = error - previous_error_;
    d *= parameters_.kd.get();

    // Backup previous error
    previous_error_ = error;

    return p + i + d;
}

} // namespace pid

} // namespace cogip
