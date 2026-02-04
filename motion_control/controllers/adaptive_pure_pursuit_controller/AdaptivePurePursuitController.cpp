// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup    adaptive_pure_pursuit_controller
/// @{
/// @file
/// @brief      Adaptive Pure Pursuit controller implementation
/// @author     Gilles DOFFE <g.doffe@gmail.com>

#include "adaptive_pure_pursuit_controller/AdaptivePurePursuitController.hpp"
#include "path/MotionDirection.hpp"
#include "path/Path.hpp"
#include "pose_straight_filter/PoseStraightFilterParameters.hpp" // for target_pose_status_t
#include "trigonometry.h"

#include "etl/absolute.h"
#include "etl/algorithm.h"

#include <cmath>

#define ENABLE_DEBUG 0
#include <debug.h>
#include <log.h>

namespace cogip {

namespace motion_control {

void AdaptivePurePursuitController::execute(ControllersIO& io)
{
    DEBUG("AdaptivePurePursuitController::execute\n");

    // Get path singleton
    path::Path& path = path::Path::instance();
    if (!path.is_started() || path.empty()) {
        DEBUG("[PP] Path not started or empty (started=%d size=%u)\n", path.is_started(),
              static_cast<unsigned>(path.size()));
        // Output zero speeds but do NOT signal pose_reached (path not ready yet)
        io.set(keys_.linear_speed_order, 0.0f);
        io.set(keys_.angular_speed_order, 0.0f);
        io.set(keys_.pose_reached, target_pose_status_t::moving);
        return;
    }

    // Read current pose
    float current_x = 0.0f;
    float current_y = 0.0f;
    float current_O = 0.0f;
    if (auto opt = io.get_as<float>(keys_.current_pose_x)) {
        current_x = *opt;
    }
    if (auto opt = io.get_as<float>(keys_.current_pose_y)) {
        current_y = *opt;
    }
    if (auto opt = io.get_as<float>(keys_.current_pose_O)) {
        current_O = *opt;
    }
    cogip_defs::Pose robot_pose(current_x, current_y, current_O);

    // Read current speed for adaptive lookahead
    float current_speed = 0.0f;
    if (auto opt = io.get_as<float>(keys_.linear_current_speed)) {
        current_speed = etl::absolute(*opt);
    }

    // Read motion direction mode from last waypoint (PP follows entire path with same direction)
    path::motion_direction motion_dir = path::motion_direction::bidirectional;
    const path::Pose* last_wp_for_dir = path.waypoint_at(path.size() - 1);
    if (last_wp_for_dir) {
        motion_dir = last_wp_for_dir->get_motion_direction();
    }

    // Detect new path and reset state (using member flag set by reset())
    if (needs_path_init_) {
        DEBUG("[PP] Path init after reset! size=%u\n", static_cast<unsigned>(path.size()));
        DEBUG("[PP] Path waypoints:\n");
        for (size_t i = 0; i < path.size(); i++) {
            const auto* wp = path.waypoint_at(i);
            if (wp) {
                DEBUG("[PP]   WP%u: (%.1f, %.1f, %.1f)\n", static_cast<unsigned>(i),
                      static_cast<double>(wp->x()), static_cast<double>(wp->y()),
                      static_cast<double>(wp->O()));
            }
        }
        // Reset segment index for new path
        current_segment_index_ = 0;
        direction_locked_ = false;

        // Compute target direction angle based on first waypoint and motion_direction
        const path::Pose* first_wp = path.waypoint_at(0);
        if (first_wp) {
            float dx = first_wp->x() - current_x;
            float dy = first_wp->y() - current_y;
            float angle_to_first_wp = RAD2DEG(std::atan2(dy, dx));

            // Determine direction based on motion_direction
            bool go_backward = false;
            switch (motion_dir) {
            case path::motion_direction::forward_only:
                go_backward = false;
                break;
            case path::motion_direction::backward_only:
                go_backward = true;
                break;
            case path::motion_direction::bidirectional:
            default:
                // For bidirectional: choose direction based on angle to first waypoint
                float alpha = limit_angle_deg(angle_to_first_wp - current_O);
                go_backward = (etl::absolute(alpha) > 90.0f);
                // Lock direction for the entire path
                direction_locked_ = true;
                locked_go_backward_ = go_backward;
                break;
            }

            // Compute target orientation
            if (go_backward) {
                target_direction_angle_ = limit_angle_deg(angle_to_first_wp + 180.0f);
            } else {
                target_direction_angle_ = angle_to_first_wp;
            }

            DEBUG("[PP] Init: angle_to_wp=%.1f dir=%s target_angle=%.1f\n",
                  static_cast<double>(angle_to_first_wp), go_backward ? "BWD" : "FWD",
                  static_cast<double>(target_direction_angle_));
        }

        // For BIDIRECTIONAL mode, skip initial rotation and start following immediately
        // (Pure Pursuit handles rotation naturally while moving)
        // For FORWARD_ONLY or BACKWARD_ONLY, do initial rotation to face the required direction
        if (motion_dir == path::motion_direction::bidirectional) {
            state_ = State::FOLLOWING_PATH;
        } else {
            state_ = State::ROTATING_TO_DIRECTION;
        }
        first_rotating_cycle_ = true;
        needs_path_init_ = false;
    }

    // Get final orientation and bypass_final_orientation from LAST waypoint
    // PP follows the entire path continuously, so we only care about the final destination
    const path::Pose* last_wp = path.waypoint_at(path.size() - 1);
    float target_orientation = last_wp ? last_wp->O() : current_O;
    bool bypass_final_orientation = last_wp ? last_wp->bypass_final_orientation() : false;

    // Write is_intermediate from last waypoint (PP follows entire path to end)
    if (!keys_.is_intermediate.empty() && last_wp) {
        io.set(keys_.is_intermediate, last_wp->is_intermediate());
    }

    // Compute distance to goal (used for profile generation and goal detection)
    float distance_to_goal = compute_distance_to_goal(path, robot_pose);

    // Handle ROTATING_TO_DIRECTION state (initial rotation to face travel direction)
    if (state_ == State::ROTATING_TO_DIRECTION) {
        // Compute angular error to target direction
        float angular_error = limit_angle_deg(target_direction_angle_ - current_O);

        if (etl::absolute(angular_error) < parameters_.initial_rotation_threshold()) {
            // Initial rotation complete, transition to path following
            DEBUG("[PP] Initial rotation complete! error=%.2f < tol=%.2f\n",
                  static_cast<double>(angular_error),
                  static_cast<double>(parameters_.initial_rotation_threshold()));
            state_ = State::FOLLOWING_PATH;
            first_rotating_cycle_ = true; // Reset for potential ROTATING_TO_FINAL later
        } else {
            // Output angular pose error for ProfileTracker (pose loop)
            if (!keys_.angular_pose_error.empty()) {
                io.set(keys_.angular_pose_error, angular_error);
            }

            // Signal profile recomputation on first cycle
            if (!keys_.recompute_angular_profile.empty()) {
                io.set(keys_.recompute_angular_profile, first_rotating_cycle_);
                first_rotating_cycle_ = false;
            }

            DEBUG("[PP] Rotating to direction: target=%.1f current=%.1f error=%.1f recompute=%d\n",
                  static_cast<double>(target_direction_angle_), static_cast<double>(current_O),
                  static_cast<double>(angular_error), first_rotating_cycle_ ? 1 : 0);

            // Set rotating_in_place flag for ConditionalSwitch (pose loop activation)
            if (!keys_.rotating_in_place.empty()) {
                io.set(keys_.rotating_in_place, true);
            }

            // Set linear speed to zero, angular handled by pose loop
            io.set(keys_.linear_speed_order, 0.0f);
            io.set(keys_.angular_speed_order, 0.0f);
            io.set(keys_.pose_reached, target_pose_status_t::moving);
            return;
        }
    }

    // State machine for path completion
    if (state_ == State::FOLLOWING_PATH) {
        // Check if position is reached
        if (distance_to_goal < parameters_.linear_threshold()) {
            if (bypass_final_orientation) {
                // Skip final rotation, path complete
                DEBUG("[PP] Position reached, bypass final orientation\n");
                io.set(keys_.linear_speed_order, 0.0f);
                io.set(keys_.angular_speed_order, 0.0f);
                io.set(keys_.pose_reached, target_pose_status_t::reached);
                if (!keys_.path_complete.empty()) {
                    io.set(keys_.path_complete, true);
                }
                path.stop();
                return;
            } else {
                // Transition to final rotation
                DEBUG("[PP] Position reached, starting final rotation to %.1f deg\n",
                      static_cast<double>(target_orientation));
                state_ = State::ROTATING_TO_FINAL;
                first_rotating_cycle_ = true;
            }
        }
    }

    // Set rotating_in_place flag for ConditionalSwitch (pose loop activation)
    if (!keys_.rotating_in_place.empty()) {
        io.set(keys_.rotating_in_place, (state_ == State::ROTATING_TO_FINAL));
    }

    if (state_ == State::ROTATING_TO_FINAL) {
        // Compute angular error to final orientation
        float angular_error = limit_angle_deg(target_orientation - current_O);

        if (etl::absolute(angular_error) < parameters_.angular_threshold()) {
            // Final orientation reached, path complete
            DEBUG("[PP] Final orientation reached! error=%.2f < tol=%.2f\n",
                  static_cast<double>(angular_error),
                  static_cast<double>(parameters_.angular_threshold()));

            io.set(keys_.linear_speed_order, 0.0f);
            io.set(keys_.angular_speed_order, 0.0f);
            io.set(keys_.pose_reached, target_pose_status_t::reached);
            if (!keys_.path_complete.empty()) {
                io.set(keys_.path_complete, true);
            }
            // Reset rotating flag on completion
            if (!keys_.rotating_in_place.empty()) {
                io.set(keys_.rotating_in_place, false);
            }
            path.stop();
            return;
        }

        // Output angular pose error for ProfileTracker (pose loop)
        if (!keys_.angular_pose_error.empty()) {
            io.set(keys_.angular_pose_error, angular_error);
        }

        // Signal profile recomputation on first cycle of ROTATING_TO_FINAL
        if (!keys_.recompute_angular_profile.empty()) {
            io.set(keys_.recompute_angular_profile, first_rotating_cycle_);
            first_rotating_cycle_ = false;
        }

        DEBUG("[PP] Rotating to final: error=%.1f recompute=%d\n",
              static_cast<double>(angular_error), first_rotating_cycle_ ? 1 : 0);

        // Set linear speed to zero
        io.set(keys_.linear_speed_order, 0.0f);
        // Set angular speed to zero (will be overwritten by pose loop via ConditionalSwitch)
        io.set(keys_.angular_speed_order, 0.0f);
        io.set(keys_.pose_reached, target_pose_status_t::moving);
        return;
    }

    // Compute adaptive lookahead distance
    float lookahead_d = compute_lookahead_distance(current_speed);

    // Find lookahead point
    float lookahead_x = 0.0f;
    float lookahead_y = 0.0f;
    bool found = find_lookahead_point(path, robot_pose, lookahead_d, lookahead_x, lookahead_y);

    if (!found) {
        // Fallback: use last waypoint as lookahead
        if (last_wp) {
            lookahead_x = last_wp->x();
            lookahead_y = last_wp->y();
            DEBUG("[PP] No intersection found, using last WP as lookahead\n");
        } else {
            // No valid waypoint, stop
            LOG_ERROR("[PP] No valid waypoint, stopping\n");
            io.set(keys_.linear_speed_order, 0.0f);
            io.set(keys_.angular_speed_order, 0.0f);
            io.set(keys_.pose_reached, target_pose_status_t::moving);
            return;
        }
    }

    // Compute angle to lookahead point (in robot frame)
    float dx = lookahead_x - current_x;
    float dy = lookahead_y - current_y;
    float angle_to_lookahead = RAD2DEG(std::atan2(dy, dx));        // Global frame angle
    float alpha = limit_angle_deg(angle_to_lookahead - current_O); // Robot frame angle

    // Determine if we should go backward based on motion_direction
    bool go_backward = false;
    switch (motion_dir) {
    case path::motion_direction::forward_only:
        // Always go forward
        go_backward = false;
        break;
    case path::motion_direction::backward_only:
        // Always go backward
        go_backward = true;
        break;
    case path::motion_direction::bidirectional:
    default:
        // For bidirectional: lock direction on first determination to avoid oscillation
        if (!direction_locked_) {
            // First time: determine based on angle to lookahead point
            locked_go_backward_ = (etl::absolute(alpha) > 90.0f);
            direction_locked_ = true;
            DEBUG("[PP] Direction locked: %s\n", locked_go_backward_ ? "BACKWARD" : "FORWARD");
        }
        go_backward = locked_go_backward_;
        break;
    }

    // Adjust alpha for backward motion (lookahead point is now "in front" when going backward)
    if (go_backward) {
        alpha = limit_angle_deg(alpha + 180.0f);
    }

    // Compute actual distance to lookahead point (chord)
    float chord = std::sqrt(dx * dx + dy * dy);
    if (chord < 1.0f) {
        chord = 1.0f; // Avoid division by zero
    }

    // Convert alpha to radians for calculations
    float alpha_rad = DEG2RAD(alpha);
    float sin_alpha = std::sin(alpha_rad);
    float cos_alpha = std::cos(alpha_rad);

    // If |alpha| > 90°, robot is facing away from target: rotate in place
    if (cos_alpha <= 0.0f) {
        // Use trapezoidal profile for rotation: v = sqrt(2 * decel * |error|)
        // Use half of max angular speed for smoother rotate-in-place
        float rotate_in_place_max_speed = parameters_.max_angular_speed() * 0.5f;
        float abs_alpha = etl::absolute(alpha);
        float max_speed_for_decel =
            std::sqrt(2.0f * parameters_.angular_deceleration() * abs_alpha);
        float angular_speed = etl::min(rotate_in_place_max_speed, max_speed_for_decel);

        // Apply direction
        if (alpha < 0.0f) {
            angular_speed = -angular_speed;
        }

        DEBUG("[PP] Rotating in place: alpha=%.1f speed=%.2f\n", static_cast<double>(alpha),
              static_cast<double>(angular_speed));

        io.set(keys_.linear_speed_order, 0.0f);
        io.set(keys_.angular_speed_order, angular_speed);
        io.set(keys_.pose_reached, target_pose_status_t::moving);
        return;
    }

    // Normal Pure Pursuit: compute curvature κ = 2 * sin(alpha) / chord
    float curvature = 2.0f * sin_alpha / chord;
    float abs_curvature = etl::absolute(curvature);

    // Compute linear speed based on distance (trapezoidal profile)
    // Deceleration constraint: v = sqrt(2 * decel * distance)
    float max_speed_for_decel =
        std::sqrt(2.0f * parameters_.linear_deceleration() * distance_to_goal);

    // Curvature constraint: v_max = ω_max / |κ|
    // Ensures angular speed (ω = v × κ) doesn't exceed max_angular_speed
    float max_speed_for_curvature = parameters_.max_linear_speed();
    if (abs_curvature > 1e-6f) {
        float omega_max_rad = DEG2RAD(parameters_.max_angular_speed());
        max_speed_for_curvature = omega_max_rad / abs_curvature;
    }

    // Target speed is minimum of all constraints
    float target_speed = etl::min(parameters_.max_linear_speed(), max_speed_for_decel);
    target_speed = etl::min(target_speed, max_speed_for_curvature);

    // Acceleration constraint: limit speed increase per cycle
    float linear_speed;
    if (target_speed > current_speed) {
        // Accelerating
        linear_speed = etl::min(target_speed, current_speed + parameters_.linear_acceleration());
    } else {
        // Decelerating or maintaining - use target directly (decel constraint handles it)
        linear_speed = target_speed;
    }

    // Apply direction: negative speed for backward motion
    if (go_backward) {
        linear_speed = -linear_speed;
    }

    // Compute angular speed from curvature
    // Note: curvature formula gives rad/mm, linear_speed is mm/period
    // Result is rad/period, must convert to deg/period for the system
    // Use absolute linear_speed for angular calculation (direction already in linear_speed sign)
    float angular_speed_rad = etl::absolute(linear_speed) * curvature;
    float angular_speed = RAD2DEG(angular_speed_rad);

    // Clamp angular speed (safety, should not be needed with curvature constraint)
    angular_speed = etl::clamp(angular_speed, -parameters_.max_angular_speed(),
                               parameters_.max_angular_speed());

    DEBUG("[PP] robot=(%.1f,%.1f,%.1f) LA=(%.1f,%.1f) seg=%u alpha=%.1f dir=%s\n",
          static_cast<double>(current_x), static_cast<double>(current_y),
          static_cast<double>(current_O), static_cast<double>(lookahead_x),
          static_cast<double>(lookahead_y), static_cast<unsigned>(current_segment_index_),
          static_cast<double>(alpha), go_backward ? "BWD" : "FWD");
    DEBUG("[PP] lin=%.2f tgt=%.2f curv=%.2f decel=%.2f dist=%.1f\n",
          static_cast<double>(linear_speed), static_cast<double>(target_speed),
          static_cast<double>(max_speed_for_curvature), static_cast<double>(max_speed_for_decel),
          static_cast<double>(distance_to_goal));

    // Write outputs
    io.set(keys_.linear_speed_order, linear_speed);
    io.set(keys_.angular_speed_order, angular_speed);
    io.set(keys_.pose_reached, target_pose_status_t::moving);
}

float AdaptivePurePursuitController::compute_lookahead_distance(float current_speed) const
{
    // Adaptive lookahead: increases with speed
    float lookahead = parameters_.min_lookahead_distance() +
                      etl::absolute(current_speed) * parameters_.lookahead_speed_ratio();

    // Clamp to [min, max]
    return etl::clamp(lookahead, parameters_.min_lookahead_distance(),
                      parameters_.max_lookahead_distance());
}

bool AdaptivePurePursuitController::find_lookahead_point(const path::Path& path,
                                                         const cogip_defs::Pose& robot_pose,
                                                         float lookahead_d, float& lookahead_x,
                                                         float& lookahead_y)
{
    if (path.size() < 1) {
        DEBUG("[PP] find_LA: path empty\n");
        return false;
    }

    const auto& waypoints = path.waypoints();
    float robot_x = robot_pose.x();
    float robot_y = robot_pose.y();

    DEBUG("[PP] find_LA: robot=(%.1f,%.1f) LA_dist=%.1f seg_idx=%u nb_wp=%u\n",
          static_cast<double>(robot_x), static_cast<double>(robot_y),
          static_cast<double>(lookahead_d), static_cast<unsigned>(current_segment_index_),
          static_cast<unsigned>(waypoints.size()));

    // Ensure segment index is valid
    if (current_segment_index_ >= waypoints.size()) {
        DEBUG("[PP] find_LA: seg_idx reset from %u to 0\n",
              static_cast<unsigned>(current_segment_index_));
        current_segment_index_ = 0;
        current_segment_param_ = 0.0f;
    }

    // Single loop to find lookahead point
    for (size_t i = current_segment_index_; i < waypoints.size(); i++) {
        float p1x, p1y, p2x, p2y;

        if (i == 0) {
            // Implicit segment from robot to first waypoint
            p1x = robot_x;
            p1y = robot_y;
            p2x = waypoints[0].x();
            p2y = waypoints[0].y();
        } else {
            // Normal segment from waypoint[i-1] to waypoint[i]
            p1x = waypoints[i - 1].x();
            p1y = waypoints[i - 1].y();
            p2x = waypoints[i].x();
            p2y = waypoints[i].y();
        }

        float ix, iy, t_int;
        if (circle_segment_intersection(p1x, p1y, p2x, p2y, robot_x, robot_y, lookahead_d, ix, iy,
                                        t_int)) {
            // This automatically handles:
            // - Advancing to next segment (i > current_segment_index_)
            // - Progressing on current segment (t_int > current_segment_param_)
            if (i > current_segment_index_ || t_int > current_segment_param_) {
                current_segment_index_ = i;
                current_segment_param_ = t_int;
                lookahead_x = ix;
                lookahead_y = iy;
                DEBUG("[PP] find_LA: FOUND on seg %u, t=%.2f -> LA=(%.1f,%.1f)\n",
                      static_cast<unsigned>(i), static_cast<double>(t_int), static_cast<double>(ix),
                      static_cast<double>(iy));
                return true;
            }
        }
    }

    // No intersection found - use appropriate waypoint as fallback
    DEBUG("[PP] find_LA: NO intersection, fallback seg_idx=%u\n",
          static_cast<unsigned>(current_segment_index_));

    if (current_segment_index_ == 0) {
        // Still approaching first waypoint - target WP0
        lookahead_x = waypoints[0].x();
        lookahead_y = waypoints[0].y();
        DEBUG("[PP] find_LA: fallback to WP0 (%.1f,%.1f)\n", static_cast<double>(lookahead_x),
              static_cast<double>(lookahead_y));
    } else if (current_segment_index_ < waypoints.size()) {
        // Target the end of current segment
        lookahead_x = waypoints[current_segment_index_].x();
        lookahead_y = waypoints[current_segment_index_].y();
        DEBUG("[PP] find_LA: fallback to WP%u (%.1f,%.1f)\n",
              static_cast<unsigned>(current_segment_index_), static_cast<double>(lookahead_x),
              static_cast<double>(lookahead_y));
    } else {
        // Use last waypoint
        lookahead_x = waypoints[waypoints.size() - 1].x();
        lookahead_y = waypoints[waypoints.size() - 1].y();
        DEBUG("[PP] find_LA: fallback to LAST WP (%.1f,%.1f)\n", static_cast<double>(lookahead_x),
              static_cast<double>(lookahead_y));
    }
    return true;
}

float AdaptivePurePursuitController::compute_distance_to_goal(
    const path::Path& path, const cogip_defs::Pose& robot_pose) const
{
    if (path.empty()) {
        return 0.0f;
    }

    const auto& waypoints = path.waypoints();

    // Compute remaining path length from robot position to end of path
    // This handles paths that loop back (e.g., rectangles that return to start)
    float total_distance = 0.0f;

    // Distance from robot projection on current segment to end of that segment
    size_t start_idx = (current_segment_index_ < waypoints.size()) ? current_segment_index_ : 0;

    // Use current_segment_param_ to compute remaining distance on current segment
    if (start_idx > 0 && start_idx < waypoints.size()) {
        float dx = waypoints[start_idx].x() - waypoints[start_idx - 1].x();
        float dy = waypoints[start_idx].y() - waypoints[start_idx - 1].y();
        total_distance += (1.0f - current_segment_param_) * std::sqrt(dx * dx + dy * dy);
    } else {
        float dx = waypoints[start_idx].x() - robot_pose.x();
        float dy = waypoints[start_idx].y() - robot_pose.y();
        total_distance += std::sqrt(dx * dx + dy * dy);
    }

    // Sum distances between remaining waypoints
    for (size_t i = start_idx; i + 1 < waypoints.size(); i++) {
        float dx = waypoints[i + 1].x() - waypoints[i].x();
        float dy = waypoints[i + 1].y() - waypoints[i].y();
        total_distance += std::sqrt(dx * dx + dy * dy);
    }

    return total_distance;
}

bool AdaptivePurePursuitController::circle_segment_intersection(float p1x, float p1y, float p2x,
                                                                float p2y, float cx, float cy,
                                                                float r, float& ix, float& iy,
                                                                float& t_out) const
{
    // Vector from p1 to p2
    float dx = p2x - p1x;
    float dy = p2y - p1y;

    // Vector from p1 to circle center
    float fx = p1x - cx;
    float fy = p1y - cy;

    // Quadratic equation coefficients: at^2 + bt + c = 0
    float a = dx * dx + dy * dy;
    float b = 2.0f * (fx * dx + fy * dy);
    float c = fx * fx + fy * fy - r * r;

    float discriminant = b * b - 4.0f * a * c;

    if (discriminant < 0.0f || a < 1e-6f) {
        // No intersection or degenerate segment
        return false;
    }

    discriminant = std::sqrt(discriminant);

    // Two solutions: t1 and t2
    float t1 = (-b - discriminant) / (2.0f * a);
    float t2 = (-b + discriminant) / (2.0f * a);

    // We want the intersection that is:
    // 1. On the segment (0 <= t <= 1)
    // 2. Furthest along the path (largest t value, closest to p2)
    // If both intersections are valid, we take the one ahead (t2)

    // Check t2 first (further along segment)
    if (t2 >= 0.0f && t2 <= 1.0f) {
        ix = p1x + t2 * dx;
        iy = p1y + t2 * dy;
        t_out = t2;
        return true;
    }

    // Check t1
    if (t1 >= 0.0f && t1 <= 1.0f) {
        ix = p1x + t1 * dx;
        iy = p1y + t1 * dy;
        t_out = t1;
        return true;
    }

    return false;
}

} // namespace motion_control

} // namespace cogip

/// @}
