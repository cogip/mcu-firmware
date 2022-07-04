// Copyright (C) 2021 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup     lib_path
/// @{
/// @file
/// @brief       Path Pose declaration
/// @author      Eric Courtois <eric.courtois@gmail.com>

#pragma once

#include "cogip_defs/Pose.hpp"
#include "utils.hpp"

#include <vector>

namespace cogip {

namespace path {

/// Position class as used in Path
class Pose : public cogip_defs::Pose {
public:
    /// Constuctor.
    Pose(
        double x=0.0,                   ///< [in] X coordinate
        double y=0.0,                   ///< [in] Y coodinate
        double O=0.0,                   ///< [in] 0-orientation
        double max_speed_linear=0.0,    ///< [in] max speed linear
        double max_speed_angular=0.0,   ///< [in] max speed angular
        bool allow_reverse=true,        ///< [in] reverse mode
        func_cb_t act_=nullptr          ///< [in] action callback
        ) : cogip_defs::Pose(x, y, O), max_speed_linear_(max_speed_linear), max_speed_angular_(max_speed_angular), allow_reverse_(allow_reverse), act_(act_) {};

    /// Destructor
    virtual ~Pose() {};

    /// Retourn max speed linear.
    virtual double max_speed_linear() const { return max_speed_linear_; };

    /// Retourn max speed angular.
    virtual double max_speed_angular() const { return max_speed_angular_; };

    /// Is reverse mode allowed or not.
    virtual bool allow_reverse() const { return allow_reverse_; };

    /// Enable or disable reverse mode.
    virtual void set_allow_reverse(
        bool enable               ///< new value for reverse mode
        ) { allow_reverse_ = enable; };

    /// Execute action callback if set.
    virtual void act() const { if (act_) act_(); }

protected:
    double max_speed_linear_;     ///< max speed
    double max_speed_angular_;    ///< max speed
    bool allow_reverse_;          ///< reverse mode
    func_cb_t act_;               ///< action callback
};

using Poses = std::vector<cogip::path::Pose>;

} // namespace path

} // namespace cogip

/// @}
