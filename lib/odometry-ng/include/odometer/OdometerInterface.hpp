#pragma once

#include <cstdint>

#include "cogip_defs/Polar.hpp"
#include "cogip_defs/Pose.hpp"

namespace cogip {

namespace odometer {

class OdometerInterface {

// Internal structures for easier Position and Velocity management
struct Position {
    Position() : x(0), y(0), O(0) {}
    Position(float x, float y, float O) : x(x), y(y), O(O) {}

    float x, y, O;
};

struct Velocity {
    Velocity() : linear(0), angular(0) {}
    Velocity(float linear, float angular) : linear(linear), angular(angular) {}

    float linear, angular;
};

public:
    //// @brief Set the default odometry position
    ///
    /// @note this function should be called to reset robot position and defined a new default one
    ///
    /// @param x X coordinate (mm)
    /// @param y Y coordinate (mm)
    /// @param O angle (deg)
    virtual void set_position(double x, double y, double O) = 0;

    /// @brief Set the default odometry position
    ///
    /// @note this function should be called to reset robot position and defined a new default one
    ///
    /// @param pos postion reference
    virtual void set_position(const cogip::cogip_defs::Pose &pos) = 0;

    /// @brief Copy current position using cogip def format
    /// @note Data units:
    ///         - x, y: mm
    ///         - O: deg
    /// @param pose cogip::cogip_defs::Pose current pose
    virtual void copy_position(cogip::cogip_defs::Pose &pose) = 0;

    /// @brief Copy current velocities using cogip def format
    /// @note Data units:
    ///         - linear: mm/s
    ///         - O: deg/s
    /// @param vel cogip::cogip_defs::Polar current velocities
    virtual void copy_velocities(cogip::cogip_defs::Polar &vel) = 0;

    /// @brief update new robot pose (x, y, O)
    /// @return int 0 on success, negative on failure.
    virtual int update() = 0;

protected:
    Position pos_;  /// Internal position
    Velocity vel_;  /// Internal velocity
};

} /// namespace odometer

} ///namespace cogip
