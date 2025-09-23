// Project includes
#include "pid/PID.hpp"
#include "etl/algorithm.h"

namespace cogip {

namespace pid {

float PID::compute(float error)
{
    float p, i, d;

    // Integral term is the error sum
    integral_term_ += error;

    // Integral limitation
    integral_term_ = etl::min(integral_term_, integral_term_limit_);
    integral_term_ = etl::max(integral_term_, -integral_term_limit_);

    // Proportional
    p = error * kp_;

    // Integral
    i = integral_term_ * ki_;

    // Derivative
    d = error - previous_error_;
    d *= kd_;

    // Backup previous error
    previous_error_ = error;

    return p + i + d;
}

} // namespace pid

} // namespace cogip
