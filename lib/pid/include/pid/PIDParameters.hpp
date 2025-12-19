// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

#pragma once

#include "parameter/ParameterInterface.hpp"

namespace cogip {

namespace pid {

using namespace cogip::parameter;

struct PIDParameters
{
    /// @brief Constructs the PID parameter aggregation structure
    /// @param kp Proportional gain parameter
    /// @param ki Integral gain parameter
    /// @param kd Derivative gain parameter
    /// @param integral_term_limit Integral term saturation limit parameter
    PIDParameters(const ParameterInterface<float>& kp, const ParameterInterface<float>& ki,
                  const ParameterInterface<float>& kd,
                  const ParameterInterface<float>& integral_term_limit)
        : kp(kp), ki(ki), kd(kd), integral_term_limit(integral_term_limit)
    {
    }

    /// Read-only parameters. Each parameter has its own getter.
    const ParameterInterface<float>& kp;
    const ParameterInterface<float>& ki;
    const ParameterInterface<float>& kd;
    const ParameterInterface<float>& integral_term_limit;
};

} // namespace pid

} // namespace cogip
