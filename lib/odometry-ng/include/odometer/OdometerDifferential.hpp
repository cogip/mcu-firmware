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
    explicit OdometerDifferential(OdometerDifferentialParameters &parameters) : parameters_(parameters), left_encoder_(NULL), right_encoder_(NULL) {}

    /// @brief Set the default odometry position
    ///
    /// @note this function should be called to reset robot position and defined a new default one
    ///
    /// @param x X coordinate (mm)
    /// @param y Y coordinate (mm)
    /// @param O angle (deg)
    void set_position(double x, double y, double O) override
    {
        pos_.x = x;
        pos_.y = y;
        pos_.O = DEG2RAD(O);
    }
    

    /// @brief Set the default odometry position
    ///
    /// @note this function should be called to reset robot position and defined a new default one
    ///
    /// @param pos postion reference
    void set_position(const cogip::cogip_defs::Pose &pos) override
    {
        pos_.x = pos.coords().x();
        pos_.y = pos.coords().y();
        pos_.O = DEG2RAD(pos.O());
    }
    
    /// @brief Copy current position using cogip def format
    /// @note Data units:
    ///         - x, y: mm
    ///         - O: deg
    /// @param pose cogip::cogip_defs::Pose current pose
    void copy_position(cogip::cogip_defs::Pose &pose) override
    { 
        pose.set_x(pos_.x);
        pose.set_y(pos_.y);
        pose.set_O(RAD2DEG(pos_.O));
    }
    
    /// @brief Copy current velocities using cogip def format
    /// @note Data units:
    ///         - linear: mm/s
    ///         - O: deg/s
    /// @param vel cogip::cogip_defs::Polar current velocities
    void copy_velocities(cogip::cogip_defs::Polar &vel) override
    { 
        vel.set_distance(vel_.linear);
        vel.set_angle(RAD2DEG(vel_.angular));
    }

    /// @brief Setup the encoders objects
    /// @param left_encoder Left encoder
    /// @param right_encoder Right encoder
    void setup(cogip::encoder::EncoderInterface &left_encoder,
               cogip::encoder::EncoderInterface &right_encoder)
    {
        left_encoder_  = &left_encoder;
        right_encoder_ = &right_encoder;
    }
    
    /// @brief update new robot pose (x, y, O)
    /// @return int 0 on success, negative on failure.
    int update() override;

private:
    OdometerDifferentialParameters &parameters_;

    cogip::encoder::EncoderInterface *left_encoder_;
    cogip::encoder::EncoderInterface *right_encoder_;
};

} // namespace odometer

} // namespace cogip