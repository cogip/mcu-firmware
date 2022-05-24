#include "planners/astar/AstarPlanner.hpp"

// RIOT includes
#include "riot/chrono.hpp"
#include "riot/thread.hpp"

// Project includes
#include "app.hpp"
#include "avoidance.hpp"
#include "platform.hpp"
#include "tracefd/tracefd.hpp"

// Enable or disable debug for this file only
#define ENABLE_DEBUG        (0)
#include "debug.h"

// Periodic task
#define TASK_PERIOD_MS      (100)

namespace cogip {

namespace planners {

int AstarPlanner::trajectory_get_route_update(const cogip::cogip_defs::Pose &robot_pose)
{
    // Current final path pose to reach
    const cogip::path::Pose *current_path_pos = path_.current_pose();
    // Pose to reach by the controller
    cogip::cogip_defs::Pose pose_to_reach = ctrl_get_pose_to_reach(ctrl_);
    // Avoidance graph position index. This index increments on each
    // intermediate pose used to reach the current path pose
    static int avoidance_index = 1;
    // Avoidance graph computing status
    bool avoidance_status = false;
    // Control variable if there is still at least a reachable pose in the path
    int nb_pose_reachable = 0;
    // Recompute avoidance graph if not 0
    bool avoidance_update = avoidance_check_recompute(robot_pose,
                                                      pose_to_reach);

    // Ask to the controller if the targeted position has been reached
    if (ctrl_is_pose_reached(ctrl_)) {
        // If the targeted pose has been reached, check if it is the current path pose
        if ((pose_to_reach.x() == current_path_pos->x())
            && (pose_to_reach.y() == current_path_pos->y())) {
            //  If the targeted pose is the current path pose to reach, launch
            //  the action and update the current path pose to reach to the next
            //  one in path, if allowed.
            DEBUG("planner: Controller has reached final position.\n");

            // If in automatic planner mode
            if (allow_change_path_pose_) {
                // If it exists launch action associated to the point and
                // increment the position to reach in the path
                DEBUG("planner: action launched!\n");
                current_path_pos->act();
                DEBUG("planner: action finished!\n");
                DEBUG("planner: Increment position.\n");
                path_.next();
            }
            // Update current path targeted position in case it has changed
            current_path_pos = path_.current_pose();

            // As current path pose could have changed, avoidance graph needs
            // to be recomputed
            avoidance_update = true;
        }
        else if ((!allow_change_path_pose_)
                 && (!ctrl_is_pose_intermediate(ctrl_))) {
            // Update current path targeted position in case it has changed
            current_path_pos = path_.current_pose();
            avoidance_update = true;
        }
        else if (avoidance_check_recompute(robot_pose,
                                           *current_path_pos)) {
            DEBUG("planner: Need to recompute avoidance graph.\n");
            avoidance_update = true;
        }
        // If it is an intermediate pose, just go to the next one in
        // avoidance graph
        else {
            DEBUG("planner: Controller has reached intermediate position.\n");
            avoidance_index++;
        }
    }

    // If the robot is blocked
    if (ctrl_->control.current_mode == CTRL_MODE_BLOCKED) {
        DEBUG("planner: Controller is blocked.\n");
        if (!allow_change_path_pose_) {
            goto trajectory_get_route_update_error;
        }
        // Increment the position to reach in the path
        path_.unreachable();
        current_path_pos = path_.current_pose();
        // As current path pose has changed, avoidance graph needs to be recomputed
        avoidance_update = true;
    }

    // If the avoidance graph needs to be recomputed
    if (avoidance_update) {
        DEBUG("planner: Updating graph!\n");
        // Update the avoidance graph according to the current robot position
        // and the path target position.
        // Updating the graph results in a new array of intermediate positions
        // to reach the targeted path current position.
        // The current intermediate position is then pointed by avoidance_index
        avoidance_status = avoidance_build_graph(robot_pose,
                                                 *current_path_pos);

        // In case the targeted position is not reachable, select the
        // next position in the path. If no position is reachable, return an error
        nb_pose_reachable = path_.size();
        while ((!avoidance_status) && (nb_pose_reachable-- > 0)) {
            if (!avoidance_status) {
                if (!allow_change_path_pose_) {
                    goto trajectory_get_route_update_error;
                }
                path_.unreachable();
                if (*current_path_pos == *path_.current_pose()) {
                    goto trajectory_get_route_update_error;
                }
                current_path_pos = path_.current_pose();
                avoidance_status = avoidance_build_graph(robot_pose,
                                                         *current_path_pos);
            }
        }
        if (nb_pose_reachable < 0) {
            DEBUG("planner: No position reachable!\n");
            goto trajectory_get_route_update_error;
        }

        avoidance_index = 1;
    }

    // At this point the avoidance graph is valid and the next targeted
    // point can be retrieved.
    pose_to_reach.set_coords(avoidance_get_pose(avoidance_index));

    // Check if the point to reach is the current path position to reach
    if ((pose_to_reach.x() == current_path_pos->x())
        && (pose_to_reach.y() == current_path_pos->y())) {
        DEBUG("planner: Reaching final position\n");
        pose_to_reach.set_O(current_path_pos->O());
        ctrl_set_pose_intermediate(ctrl_, false);
    }
    else {
        DEBUG("planner: Reaching intermediate position\n");
        ctrl_set_pose_intermediate(ctrl_, true);
    }

    ctrl_set_pose_to_reach(ctrl_, pose_to_reach);

    return 0;

trajectory_get_route_update_error:
    return -1;
}

void *AstarPlanner::task_planner()
{
    cogip::cogip_defs::Polar speed_order;

    cogip::tracefd::out.logf("Game planner starting");

    path_.reset_current_pose_index();

    // Object context initialisation
    const cogip::path::Pose *current_path_pos = path_.current_pose();
    const cogip::cogip_defs::Pose *initial_pose = current_path_pos;
    ctrl_set_pose_current(ctrl_, *initial_pose);
    ctrl_set_speed_order(ctrl_, speed_order);
    ctrl_set_pose_reached(ctrl_);

    cogip::tracefd::out.logf("Game planner started");

    for (;;) {
        riot::time_point loop_start_time = riot::now();

        if (!started_) {
            goto yield_point;
        }

        current_path_pos = path_.current_pose();

        // Update speed order to max speed defined value in the new point to reach
        speed_order.set_distance(path_.current_max_speed_linear());
        speed_order.set_angle(path_.current_max_speed_angular());

        // reverse gear selection is granted per point to reach, in path
        ctrl_set_allow_reverse(ctrl_, current_path_pos->allow_reverse());

        if (trajectory_get_route_update(ctrl_get_pose_current(ctrl_))) {
            ctrl_set_mode(ctrl_, CTRL_MODE_STOP);
        }
        else {
            ctrl_set_mode(ctrl_, CTRL_MODE_RUNNING);
        }

        ctrl_set_speed_order(ctrl_, speed_order);

yield_point:
        riot::this_thread::sleep_until(loop_start_time += std::chrono::milliseconds(TASK_PERIOD_MS));
    }

    return 0;
}

} // namespace planners

} // namespace cogip
