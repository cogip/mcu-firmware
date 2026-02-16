// Copyright (C) 2023 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup     lib_pid
/// @{
/// @file
/// @brief       PID implementation
/// @author      Gilles DOFFE <g.doffe@gmail.com>

#pragma once

#include <etl/limits.h>

#include "PIDParameters.hpp"

namespace cogip {

namespace pid {

/// PID
class PID
{
  public:
    /// Constructor.
    explicit PID(const PIDParameters& parameters)
        : parameters_(parameters), integral_term_(0), previous_error_(0){};

    /// Reset integral term and previous_error.
    void reset()
    {
        integral_term_ = 0;
        previous_error_ = 0;
    }

    /// Compute PID.
    float compute(float error);

  private:
    const PIDParameters& parameters_;
    float integral_term_;  ///< error sum
    float previous_error_; ///< previous sum
};

} // namespace pid

} // namespace cogip

/// @}
