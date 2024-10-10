#pragma once

#include <cstdint>

#include "EncoderInterface.hpp"
#include "OdometryParams.hpp"
#include "cogip_defs/Polar.hpp"
#include "cogip_defs/Pose.hpp"

// #define SEGMENT 0
// #define ARC     1
namespace cogip {

namespace odometry {

class Odometry {

// Internal structure for easier Position and Velocity management
struct Position {
    Position() : x(0), y(0), O(0) {}
    Position(float x, float y, float O) : x(x), y(y), O(theta) {}

    float x, y, O;
};

struct Velocity {
    Velocity() : linear(0), angular(0) {}
    Velocity(float linear, float angular) : linear(linear), angular(angular) {}

    float linear, angular;
};

public:
    explicit Odometry(OdometryParams &parameters) : parameters_(parameters), left_encoder_(NULL), right_encoder_(NULL) {}

    /**
     * @brief Set the default odometry position
     *
     * @note this function should be called to reset robot position and defined a new default one
     * 
     * @param x X coordinate (mm)
     * @param y Y coordinate (mm)
     * @param O angle (deg)
     */
    void set_position(double x, double y, double O)
    {
        pos_.x = x;
        pos_.y = y;
        pos_.O = O;
    }
    

    /**
     * @brief Set the default odometry position
     *      
     * @note this function should be called to reset robot position and defined a new default one
     * 
     * @param pos postion reference
     */
    void set_position(cogip::cogip_defs::Pose &pos)
    {
        pos_.x = pos.coords().x();
        pos_.y = pos.coords().y();
        pos_.O = pos.O();
    }

    /**
     * @brief Setup the encoders objects
     *
     * @param left_encoder Left encoder
     * @param right_encoder Right encoder
     */
    void setup(cogip::encoder::EncoderInterface &left_encoder,
               cogip::encoder::EncoderInterface &right_encoder)
    {
        left_encoder_  = &left_encoder;
        right_encoder_ = &right_encoder;
    }
    
    int update(cogip::cogip_defs::Pose &current_pos, cogip::cogip_defs::Polar &current_speed);

private:
    OdometryParams &parameters_;

    cogip::encoder::EncoderInterface *left_encoder_;
    cogip::encoder::EncoderInterface *right_encoder_;

    Position pos_;
    Velocity vel_;
};

} // namespace odometry

} // namespace cogip