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

#include "PB_PathPose.hpp"

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
        double max_speed_ratio_linear=0.0,    ///< [in] max speed linear
        double max_speed_ratio_angular=0.0,   ///< [in] max speed angular
        bool allow_reverse=true,        ///< [in] reverse mode
        func_cb_t act_=nullptr          ///< [in] action callback
        );

    /// Destructor
    virtual ~Pose() {};

    /// Retourn max speed linear.
    virtual double max_speed_ratio_linear() const { return max_speed_ratio_linear_; };

    /// Retourn max speed angular.
    virtual double max_speed_ratio_angular() const { return max_speed_ratio_angular_; };

    /// Is reverse mode allowed or not.
    virtual bool allow_reverse() const { return allow_reverse_; };

    /// Enable or disable reverse mode.
    virtual void set_allow_reverse(
        bool enable               ///< new value for reverse mode
        ) { allow_reverse_ = enable; };

    /// Execute action callback if set.
    virtual void act() const { if (act_) act_(); }

    /// Initialize the object from a Protobuf message.
    void pb_read(
        const PB_PathPose &path_pose  ///< [in] Protobuf message to read
    );

    /// Copy object to a Probobuf message.
    void pb_copy(
        PB_PathPose &path_pose    ///< [out] Protobuf message to fill
        ) const;

    /// Override operator ==
    bool operator==(const Pose& other) {
        return (
            areDoublesEqual(x(), other.x()) &&
            areDoublesEqual(y(), other.y()) &&
            areDoublesEqual(O(), other.O()) &&
            areDoublesEqual(max_speed_ratio_linear(), other.max_speed_ratio_linear()) &&
            areDoublesEqual(max_speed_ratio_angular(), other.max_speed_ratio_angular()) &&
            allow_reverse() == other.allow_reverse()
        );
    };

private:
    double max_speed_ratio_linear_;     ///< max speed
    double max_speed_ratio_angular_;    ///< max speed
    bool allow_reverse_;          ///< reverse mode
    func_cb_t act_;               ///< action callback
};

} // namespace path

} // namespace cogip

/// @}
