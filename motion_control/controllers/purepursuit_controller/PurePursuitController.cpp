// Copyright (C) 2026 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup
/// @{
/// @file
/// @brief      PurePursuit controller implementation
/// @author

#include "purepursuit_controller/PurePursuitController.hpp"
#include "path/MotionDirection.hpp"
#include "trigonometry.h"

#include "etl/absolute.h"

#include <cmath>

#define ENABLE_DEBUG 0
#include <debug.h>
#include <log.h>

namespace cogip {

namespace motion_control {

void PurePursuitController::reset()
{
    linear_speed_order_ = 0.0f;
    angular_speed_order_ = 0.0f;
    linear_pose_error_ = 0.0f;
    angular_pose_error_ = 0.0f;

    current_segment_index_ = 0;
    current_segment_param_ = 0.0f;
    pose_reached_ = target_pose_status_t::reached;

    state_ = State::WAITING_FOR_PATH;
}

void PurePursuitController::execute(ControllersIO& io)
{
    DEBUG("PurePursuitController::execute\n");

    if (!path_.is_started() || path_.empty()) {
        DEBUG("[PP] Path not started or empty (started=%d size=%u)\n", path_.is_started(),
              static_cast<unsigned>(path_.size()));
        write_outputs(io, true);
        return;
    }

    // Should reset pids and ramp filers
    bool reset = false;

    // Get current pose
    current_x_ = io.get_as<float>(keys_.current_pose_x).value();
    current_y_ = io.get_as<float>(keys_.current_pose_y).value();
    current_h_ = DEG2RAD(io.get_as<float>(keys_.current_pose_O).value());

    // Internal state machine
    switch (state_) {
    case State::FOLLOWING_PATH:
        // Try to find a lookahead point on the path, fallback to projection
        if (!find_lookahead_point()) {
            LOG_WARNING("[PP] No lookahead intersection, falling back to projection\n");
            find_projection_point();
        }

        // Compute path following orders
        if (computePathOrders()) {
            LOG_INFO("[PP] Reached end of path, switching to ROTATING_TO_FINAL pos=(%.1f, %.1f, "
                     "%.1f°)\n",
                     current_x_, current_y_, RAD2DEG(current_h_));
            state_ = State::ROTATING_TO_FINAL;
            reset = true;
        }
        break;

    case State::ROTATING_TO_FINAL:
        if (computeFinalHeadingOrders()) {
            LOG_INFO("[PP] Switching to WAITING_FOR_PATH, pose marked as reached pos=(%.1f, %.1f, "
                     "%.1f°)\n\n",
                     current_x_, current_y_, RAD2DEG(current_h_));
            state_ = State::WAITING_FOR_PATH;
            reset = true;
            pose_reached_ = target_pose_status_t::reached;
            path_.stop();
        }
        break;

    case State::ROTATING_TO_INITIAL:
        // Compute carrot to know which direction to face
        if (!find_lookahead_point()) {
            find_projection_point();
        }
        if (computeInitialHeadingOrders()) {
            LOG_INFO("[PP] Initial rotation done, switching to FOLLOWING_PATH heading=%.1f°\n",
                     RAD2DEG(current_h_));
            state_ = State::FOLLOWING_PATH;
            reset = true;
        }
        break;

    case State::WAITING_FOR_PATH:
    default:
        if (path_.is_started() && !path_.empty()) {
            // Insert current pose at the front of the path to create the first segment between the
            // robot position and the first waypoint.
            path::Pose current_pose(current_x_, current_y_, RAD2DEG(current_h_));
            path_.insert_front(current_pose);

            LOG_INFO("[PP] Path started (size=%u), switching to ROTATING_TO_INITIAL\n",
                     static_cast<unsigned>(path_.size()));
            state_ = State::ROTATING_TO_INITIAL;
            pose_reached_ = target_pose_status_t::moving;
        }
        break;
    }

    // Write pose error, speed orders and pose reached state
    write_outputs(io, reset);
}

void PurePursuitController::write_outputs(ControllersIO& io, bool reset)
{
    // On reset, zero all orders and errors to ensure clean state for PID and ramp filters
    if (reset) {
        io.set(keys_.linear_speed_order, 0.0f);
        io.set(keys_.angular_speed_order, 0.0f);
        io.set(keys_.linear_pose_error, 0.0f);
        io.set(keys_.angular_pose_error, 0.0f);
    } else {
        io.set(keys_.linear_speed_order, linear_speed_order_);
        io.set(keys_.angular_speed_order, static_cast<float>(RAD2DEG(angular_speed_order_)));
        io.set(keys_.linear_pose_error, linear_pose_error_);
        io.set(keys_.angular_pose_error, angular_pose_error_);
    }

    io.set(keys_.pose_reached, pose_reached_);
    io.set(keys_.linear_pid_reset, reset);
    io.set(keys_.angular_pid_reset, reset);
    io.set(keys_.linear_ramp_reset, reset);
    io.set(keys_.angular_ramp_reset, reset);
}

bool PurePursuitController::computePathOrders()
{
    const float direction = getDirection();
    const float distance_to_goal = compute_distance_to_goal();

    // Compute the norm and the argument of the vector going from the robot to its
    // goal.
    const float chord = std::sqrt((carrot_x_ - current_x_) * (carrot_x_ - current_x_) +
                                  (carrot_y_ - current_y_) * (carrot_y_ - current_y_));
    const float delta = atan2(carrot_y_ - current_y_, carrot_x_ - current_x_) - current_h_ +
                        etl::math::pi / 2 * (1 - direction);

    // The minimum curvature that the robot must follow is `2 * sin(delta) /
    // chord`. So we deduce from this and the maximum linear velocity allowed the
    // maximum angular velocity setpoint. If it exceeds the maximum angular
    // velocity allowed, then we do the opposite: we use the maximum angular
    // velocity allowed to compute the maximum linear velocity setpoint.
    float new_linear_speed = parameters().max_linear_speed.get();
    float new_angular_speed = static_cast<float>(DEG2RAD(parameters().max_angular_speed.get()));
    if (new_angular_speed * chord >= new_linear_speed * etl::absolute(2 * std::sin(delta))) {
        new_angular_speed = new_linear_speed * etl::absolute(2 * std::sin(delta)) / chord;
    } else {
        new_linear_speed = new_angular_speed * chord / etl::absolute(2 * std::sin(delta));
    }

    // Get position control gains
    const float linear_kp = parameters().linear_kp.get();
    const float angular_kp = parameters().angular_kp.get();

    // Compute pose orders
    const float linear_pose_order = (chord + distance_to_goal) * direction;
    const float angular_pose_order = limit_angle_rad(delta);

    // Store pose errors for deceleration filters
    linear_pose_error_ = linear_pose_order;
    angular_pose_error_ = static_cast<float>(RAD2DEG(angular_pose_order));

    // Compute speed orders
    linear_speed_order_ =
        saturate(linear_kp * linear_pose_order, -new_linear_speed, new_linear_speed);
    angular_speed_order_ =
        saturate(angular_kp * angular_pose_order, -new_angular_speed, new_angular_speed);

    // When traveling on the path, we slow down the robot if its orientation is
    // farther than it should. The following is an empiric but continuous formula.
    if (std::cos(delta) > 0.0f) {
        linear_speed_order_ *= (1.0f + std::cos(2.0f * delta)) / 2.0f;
    } else {
        linear_speed_order_ *= 0.0f;
    }

    if (etl::absolute(chord + distance_to_goal) < parameters().linear_threshold.get()) {
        return true;
    }

    return false;
}

bool PurePursuitController::computeInitialHeadingOrders()
{
    const float direction = getDirection();
    const float target_h =
        atan2(carrot_y_ - current_y_, carrot_x_ - current_x_) + etl::math::pi / 2 * (1 - direction);

    const float max_angular_speed =
        static_cast<float>(DEG2RAD(parameters().max_angular_speed.get()));
    const float angular_kp = parameters().angular_kp.get();

    const float angular_pose_order = limit_angle_rad(target_h - current_h_);

    angular_pose_error_ = static_cast<float>(RAD2DEG(angular_pose_order));
    linear_pose_error_ = 0.0f;
    linear_speed_order_ = 0.0f;
    angular_speed_order_ =
        saturate(angular_kp * angular_pose_order, -max_angular_speed, max_angular_speed);

    const float angular_threshold_rad = DEG2RAD(parameters().angular_threshold.get());
    if (etl::absolute(angular_pose_order) < angular_threshold_rad) {
        return true;
    }
    return false;
}

bool PurePursuitController::computeFinalHeadingOrders()
{
    // Get list of waypoints
    const auto& waypoints = path_.waypoints();

    const float target_x = waypoints.back().x();
    const float target_y = waypoints.back().y();
    const float target_h = DEG2RAD(waypoints.back().O());

    /* Get max speed */
    const float max_linear_speed = parameters().max_linear_speed.get();
    const float max_angular_speed =
        static_cast<float>(DEG2RAD(parameters().max_angular_speed.get()));

    const float linear_kp = parameters().linear_kp.get();
    const float angular_kp = parameters().angular_kp.get();

    // Determine position deltas
    const float dx = target_x - current_x_;
    const float dy = target_y - current_y_;

    // Compute pose orders
    const float linear_pose_order = std::cos(current_h_) * dx + std::sin(current_h_) * dy;
    const float angular_pose_order = limit_angle_rad(target_h - current_h_);

    // Store pose errors for deceleration filters
    linear_pose_error_ = linear_pose_order;
    angular_pose_error_ = static_cast<float>(RAD2DEG(angular_pose_order));

    // Compute speed orders
    linear_speed_order_ =
        saturate(linear_kp * linear_pose_order, -max_linear_speed, max_linear_speed);
    angular_speed_order_ =
        saturate(angular_kp * angular_pose_order, -max_angular_speed, max_angular_speed);

    const float angular_threshold_rad = DEG2RAD(parameters().angular_threshold.get());
    if (etl::absolute(angular_pose_order) < angular_threshold_rad) {
        return true;
    }

    return false;
}

bool PurePursuitController::find_lookahead_point()
{
    /* Check for empty path */
    if (path_.empty()) {
        return false;
    }

    // Get list of waypoints
    const auto& waypoints = path_.waypoints();
    const size_t n = waypoints.size();
    const float lookahead_d = parameters().lookahead_distance.get();

    // Search all remaining segments from the current one onward.
    // Segment i = waypoints[i] -> waypoints[i+1] for i < n-1
    // Segment n-1 = virtual extension from waypoints[n-1] in direction O()
    for (size_t i = current_segment_index_; i < n; i++) {
        if (i < n - 1) {
            // Normal segment from waypoints[i] to waypoints[i+1]
            float p1x = waypoints[i].x();
            float p1y = waypoints[i].y();
            float p2x = waypoints[i + 1].x();
            float p2y = waypoints[i + 1].y();

            float ix, iy, t_int;
            if (circle_segment_intersection(p1x, p1y, p2x, p2y, current_x_, current_y_, lookahead_d,
                                            ix, iy, t_int)) {
                if (i > current_segment_index_ || t_int >= current_segment_param_) {
                    current_segment_index_ = i;
                    current_segment_param_ = t_int;
                    carrot_x_ = ix;
                    carrot_y_ = iy;
                    return true;
                }
            }
        } else {
            // Virtual segment: ray from last waypoint continuing the direction
            // of the last real segment (second-to-last -> last waypoint).
            float edgedx, edgedy;
            if (n >= 2) {
                edgedx = waypoints[i].x() - waypoints[i - 1].x();
                edgedy = waypoints[i].y() - waypoints[i - 1].y();
                float len = std::sqrt(edgedx * edgedx + edgedy * edgedy);
                if (len > 0.0f) {
                    edgedx /= len;
                    edgedy /= len;
                }
            } else {
                float angle = static_cast<float>(DEG2RAD(waypoints[i].O()));
                edgedx = std::cos(angle);
                edgedy = std::sin(angle);
            }

            float dx = current_x_ - waypoints[i].x();
            float dy = current_y_ - waypoints[i].y();

            float h = etl::absolute(edgedx * dy - edgedy * dx);
            if (lookahead_d < h) {
                continue;
            }

            float t = edgedx * dx + edgedy * dy;
            float t2 = t + std::sqrt(lookahead_d * lookahead_d - h * h);
            float t1 = t - std::sqrt(lookahead_d * lookahead_d - h * h);

            if (t2 < 0.0f) {
                continue;
            }
            if (t1 > 1.0f) {
                continue;
            }

            const float final_lookahead_d = parameters().final_lookahead_distance.get();
            // edgeLength = 1.0 for virtual segment, matches m_lookAheadBis / edgeLength
            if (t2 > 1.0f + final_lookahead_d) {
                t2 = 1.0f + final_lookahead_d;
            }

            if (i > current_segment_index_ || t2 >= current_segment_param_) {
                current_segment_index_ = i;
                current_segment_param_ = t2;
                carrot_x_ = waypoints[i].x() + t2 * edgedx;
                carrot_y_ = waypoints[i].y() + t2 * edgedy;
            }
            return true;
        }
    }

    return false;
}

void PurePursuitController::find_projection_point()
{
    const auto& waypoints = path_.waypoints();
    const size_t n = waypoints.size();

    // Find the closest point on the path to the robot.
    // Only iterates real segments (not virtual), matching Arduino's checkProjectionGoal().
    float min_dist = INFINITY;
    for (size_t i = current_segment_index_; i < n - 1; i++) {
        float dx = current_x_ - waypoints[i].x();
        float dy = current_y_ - waypoints[i].y();
        float edge_dx = waypoints[i + 1].x() - waypoints[i].x();
        float edge_dy = waypoints[i + 1].y() - waypoints[i].y();
        float edge_length = std::sqrt(edge_dx * edge_dx + edge_dy * edge_dy);

        float t = (edge_dx * dx + edge_dy * dy) / (edge_length * edge_length);
        if (t > 1.0f && i + 1 < n - 1) {
            continue;
        }

        float dist;
        if (t > 1.0f) {
            float dx2 = current_x_ - waypoints[i + 1].x();
            float dy2 = current_y_ - waypoints[i + 1].y();
            dist = std::sqrt(dx2 * dx2 + dy2 * dy2);
            t = 1.0f;
        } else if (t <= 0.0f) {
            dist = std::sqrt(dx * dx + dy * dy);
            t = 0.0f;
        } else {
            dist = etl::absolute(edge_dx * dy - edge_dy * dx) / edge_length;
        }

        if (dist < min_dist) {
            min_dist = dist;
            if (i > current_segment_index_ || t > current_segment_param_) {
                current_segment_index_ = i;
                current_segment_param_ = t;
            }
        }
    }

    // Compute carrot position from updated segment index and param
    size_t idx = current_segment_index_;
    float t = current_segment_param_;
    if (idx < n - 1) {
        carrot_x_ = (1.0f - t) * waypoints[idx].x() + t * waypoints[idx + 1].x();
        carrot_y_ = (1.0f - t) * waypoints[idx].y() + t * waypoints[idx + 1].y();
    } else {
        // Virtual segment: carrot = waypoint + t * direction of last segment
        float edgedx, edgedy;
        if (n >= 2) {
            edgedx = waypoints[idx].x() - waypoints[idx - 1].x();
            edgedy = waypoints[idx].y() - waypoints[idx - 1].y();
            float len = std::sqrt(edgedx * edgedx + edgedy * edgedy);
            if (len > 0.0f) {
                edgedx /= len;
                edgedy /= len;
            }
        } else {
            float angle = static_cast<float>(DEG2RAD(waypoints[idx].O()));
            edgedx = std::cos(angle);
            edgedy = std::sin(angle);
        }
        carrot_x_ = waypoints[idx].x() + t * edgedx;
        carrot_y_ = waypoints[idx].y() + t * edgedy;
    }
}

bool PurePursuitController::circle_segment_intersection(float p1x, float p1y, float p2x, float p2y,
                                                        float cx, float cy, float r, float& ix,
                                                        float& iy, float& t_out) const
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

float PurePursuitController::compute_distance_to_goal() const
{
    if (path_.empty()) {
        return 0.0f;
    }

    const auto& waypoints = path_.waypoints();
    const size_t n = waypoints.size();
    float total_distance = 0.0f;

    // Sum remaining distance from current segment onward.
    // Matches Arduino's getDistAfterGoal():
    //   segment i < n-1: waypoints[i] → waypoints[i+1]
    //   segment n-1 (virtual): unit-length extension in direction O()
    for (size_t i = current_segment_index_; i < n; i++) {
        float edgeLength;
        if (i < n - 1) {
            float dx = waypoints[i + 1].x() - waypoints[i].x();
            float dy = waypoints[i + 1].y() - waypoints[i].y();
            edgeLength = std::sqrt(dx * dx + dy * dy);
        } else {
            // Virtual segment: unit length, matching Arduino
            edgeLength = 1.0f;
        }

        if (i == current_segment_index_) {
            total_distance += (1.0f - current_segment_param_) * edgeLength;
        } else {
            total_distance += edgeLength;
        }
    }

    return total_distance;
}

float PurePursuitController::getDirection() const
{
    // Get motion direction from last target waypoint
    const auto& last_pose = path_.waypoints().back();
    switch (last_pose.get_motion_direction()) {
    case path::motion_direction::forward_only:
        return 1.0f;
    case path::motion_direction::backward_only:
        return -1.0f;
    case path::motion_direction::bidirectional:
    default:
        break;
    }

    // Bidirectional: choose optimal direction based on carrot position
    const float direction =
        std::cos(atan2(carrot_y_ - current_y_, carrot_x_ - current_x_) - current_h_);

    if (direction < 0) {
        return -1.0f;
    }

    return 1.0f;
}

float PurePursuitController::saturate(float x, float min, float max)
{
    if (x < min)
        return min;
    else if (x > max)
        return max;
    else
        return x;
}

} // namespace motion_control

} // namespace cogip

/// @}
