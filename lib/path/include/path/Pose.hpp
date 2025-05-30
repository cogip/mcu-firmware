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
        float x=0.0,                   ///< [in] X coordinate
        float y=0.0,                   ///< [in] Y coodinate
        float O=0.0,                   ///< [in] 0-orientation
        float max_speed_ratio_linear=0.0,  ///< [in] max speed linear
        float max_speed_ratio_angular=0.0, ///< [in] max speed angular
        bool allow_reverse=true,        ///< [in] reverse mode
        bool bypass_antiblocking=false, ///< [in] bypass anti blocking
        uint32_t timeout_ms=0,          ///< [in] move timeout
        bool bypass_final_orientation=false, ///< [in] bypass final orientation
        bool is_intermediate=false      ///< [in] is an intermediate pose
        );

    /// Destructor
    virtual ~Pose() {};

    /// Return max speed linear.
    virtual float max_speed_ratio_linear() const { return max_speed_ratio_linear_; };

    /// Return max speed angular.
    virtual float max_speed_ratio_angular() const { return max_speed_ratio_angular_; };

    /// Is reverse mode allowed or not.
    virtual bool allow_reverse() const { return allow_reverse_; };

    /// Enable or disable reverse mode.
    virtual void set_allow_reverse(
        bool enable               ///< new value for reverse mode
        ) { allow_reverse_ = enable; };

    /// Return true if anti blocking should be bypassed
    virtual bool bypass_anti_blocking() const { return bypass_anti_blocking_; }

    /// Return timeout to reach the pose, 0 if timeout should be disabled
    virtual uint32_t timeout_ms() const { return timeout_ms_; }

    /// Return true if final orientation should be bypassed
    virtual bool bypass_final_orientation() const { return bypass_final_orientation_; }

    /// Return true if pose is an intermediate pose
    virtual bool is_intermediate() const { return is_intermediate_; }

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
            allow_reverse() == other.allow_reverse() &&
	    bypass_anti_blocking() == other.bypass_anti_blocking() &&
            timeout_ms() == other.timeout_ms() &&
            bypass_final_orientation() == other.bypass_final_orientation()
        );
    };

private:
    float max_speed_ratio_linear_; ///< max speed
    float max_speed_ratio_angular_;///< max speed
    bool allow_reverse_;            ///< reverse mode
    bool bypass_anti_blocking_;     ///< bypass anti blocking
    uint32_t timeout_ms_;           ///< timeout(ms) to reach the path pose
    bool bypass_final_orientation_; ///< bypass anti blocking
    bool is_intermediate_;          ///< intermediate pose
};

} // namespace path

} // namespace cogip

/// @}
