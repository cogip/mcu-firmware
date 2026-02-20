// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup    adaptive_pure_pursuit_controller Adaptive Pure Pursuit controller
/// @{
/// @file
/// @brief      Adaptive Pure Pursuit controller for path following
/// @author     Gilles DOFFE <g.doffe@gmail.com>

#pragma once

// Project includes
#include "AdaptivePurePursuitControllerIOKeys.hpp"
#include "AdaptivePurePursuitControllerParameters.hpp"
#include "cogip_defs/Pose.hpp"
#include "log.h"
#include "motion_control_common/Controller.hpp"
#include "motion_control_common/ControllersIO.hpp"
#include "motion_control_common/TrapezoidalProfile.hpp"
#include "path/Path.hpp"

namespace cogip {

namespace motion_control {

/// @brief Adaptive Pure Pursuit controller for path following
///
/// This controller implements the Pure Pursuit algorithm with adaptive lookahead distance.
/// It computes linear and angular speed orders to follow a path smoothly.
///
/// **Algorithm:**
/// 1. Compute adaptive lookahead distance based on current speed
/// 2. Find the lookahead point on the path (intersection with lookahead circle)
/// 3. Compute the curvature to reach the lookahead point
/// 4. Generate linear and angular speed commands
///
/// **Adaptive lookahead:**
/// lookahead_distance = min_lookahead + speed * lookahead_ratio
/// (clamped to [min_lookahead, max_lookahead])
///
/// **Curvature calculation:**
/// curvature = 2 * sin(alpha) / lookahead_distance
/// where alpha is the angle between robot heading and lookahead point
///
/// **Speed commands:**
/// - linear_speed: based on max speed, reduced near goal
/// - angular_speed: linear_speed * curvature (clamped to max_angular_speed)
class AdaptivePurePursuitController : public Controller<AdaptivePurePursuitControllerIOKeys,
                                                        AdaptivePurePursuitControllerParameters>
{
  public:
    /// @brief Controller state for path following
    enum class State {
        ROTATING_TO_DIRECTION, ///< Rotating to face direction of travel before moving
        FOLLOWING_PATH,        ///< Following path with Pure Pursuit
        ROTATING_TO_FINAL      ///< Rotating to final orientation at end of path
    };

    /// @brief Constructor
    /// @param keys       Reference to a POD containing all controller keys
    /// @param parameters Reference to controller parameters
    /// @param name       Optional instance name for identification
    explicit AdaptivePurePursuitController(
        const AdaptivePurePursuitControllerIOKeys& keys,
        const AdaptivePurePursuitControllerParameters& parameters, etl::string_view name = "")
        : Controller<AdaptivePurePursuitControllerIOKeys, AdaptivePurePursuitControllerParameters>(
              keys, parameters, name),
          current_segment_index_(0), current_segment_param_(0.0f), direction_locked_(false),
          locked_go_backward_(false), state_(State::ROTATING_TO_DIRECTION),
          first_rotating_cycle_(true), needs_path_init_(true), target_direction_angle_(0.0f)
    {
    }

    /// @brief Get the type name of this controller
    const char* type_name() const override
    {
        return "AdaptivePurePursuitController";
    }

    /// @brief Reset internal state for new path
    void reset() override
    {
        current_segment_index_ = 0;
        current_segment_param_ = 0.0f;
        direction_locked_ = false;
        locked_go_backward_ = false;
        state_ = State::ROTATING_TO_DIRECTION;
        first_rotating_cycle_ = true;
        needs_path_init_ = true; // Force re-detection of path on next execute()
        target_direction_angle_ = 0.0f;
    }

    /// @brief Execute Pure Pursuit algorithm
    ///
    /// 1. Read current pose and speed from IO
    /// 2. Read path from IO
    /// 3. Compute adaptive lookahead distance
    /// 4. Find lookahead point on path
    /// 5. Compute curvature and speed commands
    /// 6. Write outputs to IO
    ///
    /// @param io Controller IO map for reading inputs and writing outputs
    void execute(ControllersIO& io) override;

  private:
    size_t current_segment_index_; ///< Current segment index for lookahead search optimization
    float current_segment_param_;  ///< Robot projection parameter [0,1] on current segment
    bool direction_locked_;   ///< True once direction has been determined (for bidirectional mode)
    bool locked_go_backward_; ///< Locked direction: true=backward, false=forward
    State state_;             ///< Current controller state
    bool first_rotating_cycle_;    ///< True on first cycle of rotating states (for profile
                                   ///< recomputation)
    bool needs_path_init_;         ///< True when path tracking state needs reinitialization
    float target_direction_angle_; ///< Target angle for ROTATING_TO_DIRECTION state

    /// @brief Compute adaptive lookahead distance based on current speed
    /// @param current_speed Current linear speed (mm/period)
    /// @return Lookahead distance (mm)
    float compute_lookahead_distance(float current_speed) const;

    /// @brief Find the lookahead point on the path
    /// @param path         Reference to the path
    /// @param robot_pose   Current robot pose
    /// @param lookahead_d  Lookahead distance
    /// @param[out] lookahead_x  X coordinate of lookahead point
    /// @param[out] lookahead_y  Y coordinate of lookahead point
    /// @return true if a valid lookahead point was found
    bool find_lookahead_point(const path::Path& path, const cogip_defs::Pose& robot_pose,
                              float lookahead_d, float& lookahead_x, float& lookahead_y);

    /// @brief Compute distance from robot to the end of the path
    /// @param path       Reference to the path
    /// @param robot_pose Current robot pose
    /// @return Distance to the last waypoint (mm)
    float compute_distance_to_goal(const path::Path& path,
                                   const cogip_defs::Pose& robot_pose) const;

    /// @brief Find intersection of a circle with a line segment
    /// @param p1x, p1y   Start of segment
    /// @param p2x, p2y   End of segment
    /// @param cx, cy     Center of circle (robot position)
    /// @param r          Radius of circle (lookahead distance)
    /// @param[out] ix, iy    Intersection point (closest to p2)
    /// @param[out] t_out     Parameter [0,1] of intersection on segment
    /// @return true if intersection exists
    bool circle_segment_intersection(float p1x, float p1y, float p2x, float p2y, float cx, float cy,
                                     float r, float& ix, float& iy, float& t_out) const;
};

} // namespace motion_control

} // namespace cogip

/// @}
