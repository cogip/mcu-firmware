// System includes
#include <cmath>
#include <iostream>

// Project includes
#include "cogip_defs/Pose.hpp"
#include "cogip_defs/Polar.hpp"
#include "pose_straight_filter/PoseStraightFilter.hpp"
#include "trigonometry.h"

namespace cogip {

namespace motion_control {

void PoseStraightFilter::execute(ControllersIO& io)
{
    COGIP_DEBUG_COUT("Execute PoseStraightFilter");

    // Read current pose coordinates and orientation
    float curr_x = 0.0f;
    if (auto opt = io.get_as<float>(keys_->current_pose_x)) {
        curr_x = *opt;
    }
    float curr_y = 0.0f;
    if (auto opt = io.get_as<float>(keys_->current_pose_y)) {
        curr_y = *opt;
    }
    float curr_o = 0.0f;
    if (auto opt = io.get_as<float>(keys_->current_pose_o)) {
        curr_o = *opt;
    }
    cogip_defs::Pose current_pose(curr_x, curr_y, curr_o);

    // Read target pose coordinates and orientation
    float tgt_x = 0.0f;
    if (auto opt = io.get_as<float>(keys_->target_pose_x)) {
        tgt_x = *opt;
    }
    float tgt_y = 0.0f;
    if (auto opt = io.get_as<float>(keys_->target_pose_y)) {
        tgt_y = *opt;
    }
    float tgt_o = 0.0f;
    if (auto opt = io.get_as<float>(keys_->target_pose_o)) {
        tgt_o = *opt;
    }
    cogip_defs::Pose target_pose(tgt_x, tgt_y, tgt_o);

    // Read current linear and angular speeds
    float curr_lin = 0.0f;
    if (auto opt = io.get_as<float>(keys_->current_linear_speed)) {
        curr_lin = *opt;
    }
    float curr_ang = 0.0f;
    if (auto opt = io.get_as<float>(keys_->current_angular_speed)) {
        curr_ang = *opt;
    }
    cogip_defs::Polar current_speed(curr_lin, curr_ang);

    // Read target linear and angular speeds
    float tgt_lin = 0.0f;
    if (auto opt = io.get_as<float>(keys_->target_linear_speed)) {
        tgt_lin = *opt;
    }
    float tgt_ang = 0.0f;
    if (auto opt = io.get_as<float>(keys_->target_angular_speed)) {
        tgt_ang = *opt;
    }
    cogip_defs::Polar target_speed(tgt_lin, tgt_ang);

    // Read reverse permission flag
    bool allow_rev = false;
    if (auto opt = io.get_as<float>(keys_->allow_reverse)) {
        allow_rev = static_cast<bool>(*opt);
    }

    // Persist reverse flag when switching to MOVE_TO_POSITION state
    static bool force_rev = false;
    static cogip_defs::Pose prev_target(INT32_MAX, INT32_MAX, INT32_MAX);

    if ((tgt_x != prev_target.x()) ||
        (tgt_y != prev_target.y()) ||
        (tgt_o != prev_target.O()))
    {
        force_rev = false;
        prev_target = target_pose;
    }
    if (force_rev) {
        allow_rev = true;
    }

    // Compute pose error as polar difference
    cogip_defs::Polar pos_err = target_pose - current_pose;
    if (allow_rev && (std::fabs(pos_err.angle()) > 90.0f)) {
        pos_err.reverse();
    }

    bool no_ang_limit = false;
    bool no_lin_limit = false;

    float lin_thresh = this->parameters_->linear_threshold();
    float ang_thresh = this->parameters_->angular_threshold();
    float lin_decel = this->parameters_->linear_deceleration();
    float ang_decel = this->parameters_->angular_deceleration();
    float ang_inter = this->parameters_->angular_intermediate_threshold();

    float abs_dist = std::fabs(pos_err.distance());
    float abs_ang = std::fabs(pos_err.angle());

    // State transitions
    if ((current_state_ == PoseStraightFilterState::ROTATE_TO_DIRECTION) &&
        (abs_dist <= lin_thresh))
    {
        current_state_ = PoseStraightFilterState::ROTATE_TO_FINAL_ANGLE;
    }

    switch (current_state_) {
        case PoseStraightFilterState::ROTATE_TO_DIRECTION:
            if (abs_ang > ang_inter) {
                target_speed.set_distance(0.0f);
            } else {
                target_speed.set_distance(pos_err.distance() >= 0 ? 1.0f : -1.0f); // forward motion sign
                force_rev = true;
                current_state_ = PoseStraightFilterState::MOVE_TO_POSITION;
            }
            break;

        case PoseStraightFilterState::MOVE_TO_POSITION:
            if (abs_dist <= lin_thresh) {
                current_state_ = PoseStraightFilterState::ROTATE_TO_FINAL_ANGLE;
            }
            break;

        case PoseStraightFilterState::ROTATE_TO_FINAL_ANGLE:
            if (!this->parameters_->bypass_final_orientation()) {
                pos_err.set_angle(limit_angle_deg(tgt_o - curr_o));
            } else {
                pos_err.set_angle(0.0f);
            }
            target_speed.set_distance(0.0f);
            if (std::fabs(pos_err.angle()) <= ang_thresh) {
                current_state_ = PoseStraightFilterState::FINISHED;
            }
            break;

        case PoseStraightFilterState::FINISHED:
            target_speed.set_distance(0.0f);
            target_speed.set_angle(0.0f);
            break;
    }

    // Apply deceleration rules if needed
    if (abs_dist <= ((curr_lin * curr_lin) / (2.0f * lin_decel))) {
        target_speed.set_distance(std::sqrt(2.0f * lin_decel * abs_dist));
    }
    if (abs_ang <= ((curr_ang * curr_ang) / (2.0f * ang_decel))) {
        target_speed.set_angle(std::sqrt(2.0f * ang_decel * abs_ang));
    }

    // Write linear pose error
    io.set(keys_->linear_pose_error, pos_err.distance());

    // Write linear current speed
    io.set(keys_->linear_current_speed, current_speed.distance());

    // Write linear target speed as absolute
    io.set(keys_->linear_target_speed, std::fabs(target_speed.distance()));

    // Write linear speed filter flag
    io.set(keys_->linear_speed_filter_flag, static_cast<float>(no_lin_limit));

    // Write angular pose error
    io.set(keys_->angular_pose_error, pos_err.angle());

    // Write angular current speed
    io.set(keys_->angular_current_speed, current_speed.angle());

    // Write angular target speed as absolute
    io.set(keys_->angular_target_speed, std::fabs(target_speed.angle()));

    // Write angular speed filter flag
    io.set(keys_->angular_speed_filter_flag, static_cast<float>(no_ang_limit));

    // Write updated pose reached status
    target_pose_status_t reached = (current_state_ == PoseStraightFilterState::FINISHED)
                                   ? target_pose_status_t::reached
                                   : target_pose_status_t::moving;
    io.set(keys_->pose_reached, static_cast<float>(reached));
}

}  // namespace motion_control

}  // namespace cogip
