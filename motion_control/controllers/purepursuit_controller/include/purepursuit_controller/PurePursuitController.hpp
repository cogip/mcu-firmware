// Copyright (C) 2026 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup
/// @{
/// @file
/// @brief      Pure Pursuit controller for path following
/// @author

#pragma once

// Project includes
#include "PurePursuitControllerIOKeys.hpp"
#include "PurePursuitControllerParameters.hpp"
#include "cogip_defs/Pose.hpp"
#include "log.h"
#include "motion_control_common/Controller.hpp"
#include "motion_control_common/ControllersIO.hpp"
#include "path/Path.hpp"

namespace cogip {

namespace motion_control {

class PurePursuitController
    : public Controller<PurePursuitControllerIOKeys, PurePursuitControllerParameters>
{
  public:
    /// @brief Constructor
    /// @param keys       Reference to a POD containing all controller keys
    /// @param parameters Reference to controller parameters
    /// @param path       Reference to the Path object for waypoint navigation
    /// @param name       Optional instance name for identification
    explicit PurePursuitController(const PurePursuitControllerIOKeys& keys,
                                   const PurePursuitControllerParameters& parameters,
                                   path::Path& path, etl::string_view name = "")
        : Controller<PurePursuitControllerIOKeys, PurePursuitControllerParameters>(keys, parameters,
                                                                                   name),
          path_(path)
    {
    }

    /// @brief Get the type name of this controller
    const char* type_name() const override
    {
        return "PurePursuitController";
    }

    /// @brief Reset internal state for new path
    void reset() override;

    /// @brief Execute Pure Pursuit algorithm
    /// @param io Controller IO map for reading inputs and writing outputs
    void execute(ControllersIO& io) override;

  private:
    bool computePathOrders();
    bool computeInitialHeadingOrders();
    bool computeFinalHeadingOrders();

    bool find_lookahead_point();
    void find_projection_point();
    bool circle_segment_intersection(float p1x, float p1y, float p2x, float p2y, float cx, float cy,
                                     float r, float& ix, float& iy, float& t_out) const;
    float compute_distance_to_goal() const;

    float getDirection() const;
    float saturate(float x, float min, float max);
    void write_outputs(ControllersIO& io, bool reset = false);

    enum class State {
        WAITING_FOR_PATH,
        ROTATING_TO_INITIAL,
        FOLLOWING_PATH,
        ROTATING_TO_FINAL,
    };

    /// Path manager reference
    path::Path& path_;

    /// Current pose
    float current_x_ = 0.0f;
    float current_y_ = 0.0f;
    float current_h_ = 0.0f;

    /// Carrot pose
    float carrot_x_ = 0.0f;
    float carrot_y_ = 0.0f;

    /// Speed orders
    float linear_speed_order_ = 0.0f;
    float angular_speed_order_ = 0.0f;

    /// Pose errors (for deceleration filters)
    float linear_pose_error_ = 0.0f;
    float angular_pose_error_ = 0.0f;

    /// Pose reached
    target_pose_status_t pose_reached_ = target_pose_status_t::reached;

    /// Internal
    size_t current_segment_index_ = 0;
    float current_segment_param_ = 0.0f;

    State state_ = State::WAITING_FOR_PATH;
};

} // namespace motion_control

} // namespace cogip

/// @}
