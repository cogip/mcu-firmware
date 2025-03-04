#pragma once

#include <cstdint>

#include "encoder/EncoderInterface.hpp"
#include "OdometerInterface.hpp"
#include "OdometerDifferentialParameters.hpp"

namespace cogip {

namespace odometer {

class OdometerDifferential: public OdometerInterface {

public:
    /// @brief Construct a new differential odometer object
    /// @param parameters Odometer parameters
    explicit OdometerDifferential(OdometerDifferentialParameters &parameters, 
                                  cogip::encoder::EncoderInterface &left_encoder,
                                  cogip::encoder::EncoderInterface &right_encoder) : 
                                  parameters_(parameters), left_encoder_(left_encoder), right_encoder_(right_encoder) {}

    /// @brief Set the default odometry pose
    ///
    /// @note this function should be called to reset robot pose and defined a new default one
    ///
    /// @param x X coordinate (mm)
    /// @param y Y coordinate (mm)
    /// @param O angle (deg)
    void set_pose(float x, float y, float O) override
    {
        pose_.set_x(x);
        pose_.set_y(y);
        pose_.set_O(O);
    }

    /// @brief Set the default odometry pose
    ///
    /// @note this function should be called to reset robot pose and defined a new default one
    ///
    /// @param pose postion reference
    void set_pose(const cogip::cogip_defs::Pose &pose) override
    {
        pose_.set_x(pose.x());
        pose_.set_y(pose.y());
        pose_.set_O(pose.O());
    }

    /// @brief Get current pose using cogip def format
    /// @note Data units:
    ///         - x, y: mm
    ///         - O: deg
    /// @return pose cogip::cogip_defs::Pose current pose reference
    const cogip::cogip_defs::Pose& pose() override
    {
        return pose_;
    }

    /// @brief Get current polar pose delta cogip def format
    /// @note Data units:
    ///         - linear: mm
    ///         - O: deg
    /// @return velocity cogip::cogip_defs::Polar current polar pose delta reference
    const cogip::cogip_defs::Polar& delta_polar_pose() override
    {
        return polar_;
    }
    
    /// @brief update new robot pose (x, y, O)
    /// @return int 0 on success, negative on failure.
    int update() override;

private:
    OdometerDifferentialParameters &parameters_;

    cogip::encoder::EncoderInterface &left_encoder_;
    cogip::encoder::EncoderInterface &right_encoder_;

    cogip::cogip_defs::Pose pose_;
    cogip::cogip_defs::Polar polar_;
};

} // namespace odometer

} // namespace cogip