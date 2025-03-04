#pragma once

#include <cstdint>

#include "cogip_defs/Polar.hpp"
#include "cogip_defs/Pose.hpp"

namespace cogip {

namespace odometer {

class OdometerInterface {
public:
    /// @brief Set the default odometry pose
    ///
    /// @note this function should be called to reset robot pose and defined a new default one
    ///
    /// @param x X coordinate (mm)
    /// @param y Y coordinate (mm)
    /// @param O angle (deg)
    virtual void set_pose(float x, float y, float O) = 0;

    /// @brief Set the default odometry pose
    ///
    /// @note this function should be called to reset robot pose and defined a new default one
    ///
    /// @param pose postion reference
    virtual void set_pose(const cogip::cogip_defs::Pose &pose) = 0;

    /// @brief Get current pose using cogip def format
    /// @note Data units:
    ///         - x, y: mm
    ///         - O: deg
    /// @return pose cogip::cogip_defs::Pose current pose reference
    virtual const cogip::cogip_defs::Pose& pose() = 0;

    /// @brief Get current polar pose delta cogip def format
    /// @note Data units:
    ///         - linear: mm
    ///         - O: deg
    /// @return velocity cogip::cogip_defs::Polar current polar pose delta reference
    virtual const cogip::cogip_defs::Polar& delta_polar_pose() = 0;

    /// @brief update new robot pose (x, y, O)
    /// @return int 0 on success, negative on failure.
    virtual int update() = 0;
};

} /// namespace odometer

} ///namespace cogip
