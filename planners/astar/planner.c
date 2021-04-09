#include "planner.h"

/* System includes */
#include <stdio.h>

/* RIOT includes */
#include "irq.h"
#include "periph/adc.h"
#include "log.h"
#include "xtimer.h"

/* Project includes */
#include "app.h"
#include "avoidance.h"
#include "platform.h"
#include "path.h"

#ifdef MODULE_SHELL_PLANNERS
#include "shell_planners.h"
#endif /* MODULE_SHELL_PLANNERS */

/* Enable or disable debug for this file only */
#define ENABLE_DEBUG        (0)
#include "debug.h"

static uint8_t pln_started = FALSE;

/* Planner can automatically change next path pose to reach when current pose
 * is reached */
static uint8_t allow_change_path_pose = TRUE;

/* Periodic task */
#define TASK_PERIOD_MS      (50)

void pln_set_allow_change_path_pose(uint8_t value)
{
    allow_change_path_pose = value;
}

void pln_start(ctrl_t* ctrl)
{
    ctrl_set_mode(ctrl, CTRL_MODE_RUNNING);
    pln_started = TRUE;
}

void pln_stop(ctrl_t* ctrl)
{
    ctrl_set_mode(ctrl, CTRL_MODE_STOP);
    pln_started = FALSE;
}

static int trajectory_get_route_update(ctrl_t* ctrl, const pose_t* robot_pose,
        path_t* path)
{
    /* Current final path pose to reach */
    const path_pose_t* current_path_pos = path_get_current_path_pos(path);
    /* Pose to reach by the controller */
    pose_t pose_to_reach = ctrl_get_pose_to_reach(ctrl);
    /* Avoidance graph position index. This index increments on each
     * intermediate pose used to reach the current path pose */
    static int avoidance_index = -1;
    /* Control variable if there is still at least a reachable pose in the
     * path */
    int nb_pose_reachable = 0;
    /* Recompute avoidance graph if not 0 */
    int avoidance_update = FALSE;

    /* Read sensors to know if some obstacles are detected and then if
     * avoidance graph needs to be recomputed */
    avoidance_update = pf_read_sensors();

    /* Ask to the controller if the targeted position has been reached */
    if (ctrl_is_pose_reached(ctrl)) {
        /* If the targeted pose has been reached, check if it is the
         * current path pose */
        if ((pose_to_reach.x == current_path_pos->pos.x)
            && (pose_to_reach.y == current_path_pos->pos.y)) {
            /* If the targeted pose is the current path pose to reach, launch
             * the action and update the current path pose to reach to the next
             * one in path, if allowed. */
            DEBUG("planner: Controller has reached final position.\n");

            /* If in automatic planner mode */
            if (allow_change_path_pose) {
                /* If it exists launch action associated to the point and increment
                 * the position to reach in the path */
                if (current_path_pos->act) {
                    DEBUG("planner: action launched!\n");
                    (*(current_path_pos->act))();
                    DEBUG("planner: action finished!\n");
                }
                DEBUG("planner: Increment position.\n");
                path_increment_current_pose_idx(path);
            }
            /* Update current path targeted position in case it has changed */
            current_path_pos = path_get_current_path_pos(path);

            /* As current path pose could have changed, avoidance graph needs
                * to be recomputed */
            avoidance_update = TRUE;
        }
        else if ((!allow_change_path_pose) && (!ctrl_is_pose_intermediate(ctrl))) {
            /* Update current path targeted position in case it has changed */
            current_path_pos = path_get_current_path_pos(path);
            avoidance_update = TRUE;
        }
        /* If it is an intermediate pose, just go to the next one in
         * avoidance graph */
        else {
            DEBUG("planner: Controller has reached intermediate position.\n");
            avoidance_index++;
        }
    }

    /* If the robot is blocked */
    if (ctrl->control.current_mode == CTRL_MODE_BLOCKED) {
        DEBUG("planner: Controller is blocked.\n");
        if (!allow_change_path_pose)
            goto trajectory_get_route_update_error;
        /* Increment the position to reach in the path */
        path_increment_current_pose_idx(path);
        current_path_pos = path_get_current_path_pos(path);
        /* As current path pose has changed, avoidance graph needs to be
         * recomputed */
        avoidance_update = TRUE;
    }

    /* If the avoidance graph needs to be recomputed */
    if (avoidance_update) {
        DEBUG("planner: Updating graph!\n");
        /* Update the avoidance graph according to the current robot position
         * and the path target position.
         * Updating the graph results in a new array of intermediate positions
         * to reach the targeted path current position.
         * The current intermediate position is then pointed by avoidance_index
         */
        avoidance_index = update_graph(robot_pose, &(current_path_pos->pos));

        /* In case the targeted position is not reachable, select the
         * next position in the path. If no position is reachable, return an
         * error */
        nb_pose_reachable = path->nb_pose;
        while ((avoidance_index < 0) && (nb_pose_reachable-- > 0)) {
            if (avoidance_index == -1) {
                if (!allow_change_path_pose)
                    goto trajectory_get_route_update_error;
                path_increment_current_pose_idx(path);
                if (current_path_pos == path_get_current_path_pos(path))
                    goto trajectory_get_route_update_error;
                current_path_pos = path_get_current_path_pos(path);
                avoidance_index = update_graph(robot_pose, &(current_path_pos->pos));
            }
        }
        if (nb_pose_reachable < 0) {
            DEBUG("planner: No position reachable!\n");
            goto trajectory_get_route_update_error;
        }
    }

    /* At this point the avoidance graph is valid and the next targeted
     * point can be retrieved. */
    pose_to_reach = avoidance(avoidance_index);

    /* Check if the point to reach is the current path position to reach */
    if ((pose_to_reach.x == current_path_pos->pos.x)
        && (pose_to_reach.y == current_path_pos->pos.y)) {
        DEBUG("planner: Reaching final position\n");
        pose_to_reach.O = current_path_pos->pos.O;
        ctrl_set_pose_intermediate(ctrl, FALSE);
    }
    else {
        DEBUG("planner: Reaching intermediate position\n");
        ctrl_set_pose_intermediate(ctrl, TRUE);
    }

    ctrl_set_pose_to_reach(ctrl, pose_to_reach);

    return 0;

trajectory_get_route_update_error:
    return -1;
}

void *task_planner(void *arg)
{
    (void)arg;
    pose_t initial_pose = { 0, 0, 0 };
    polar_t speed_order = { 0, 0 };
    const path_pose_t *current_path_pos = NULL;

    printf("Game planner starting\n");

    ctrl_t *ctrl = pf_get_ctrl();

    path_t* path = pf_get_path();
    if (!path) {
        printf("Machine has no path\n");
    }

    /* object context initialisation */
    path->current_pose_idx = 0;
    current_path_pos = path_get_current_path_pos(path);
    initial_pose = current_path_pos->pos;
    ctrl_set_pose_current(ctrl, &initial_pose);
    ctrl_set_speed_order(ctrl, speed_order);
    ctrl_set_pose_reached(ctrl);

    printf("Game planner started\n");

    for (;;) {
        xtimer_ticks32_t loop_start_time = xtimer_now();

        if (!pln_started) {
            goto yield_point;
        }

        current_path_pos = path_get_current_path_pos(path);

        /* Update speed order to max speed defined value in the new point to reach */
        speed_order.distance = path_get_current_max_speed(path);
        speed_order.angle = speed_order.distance / 2;

        /* reverse gear selection is granted per point to reach, in path */
        ctrl_set_allow_reverse(ctrl, current_path_pos->allow_reverse);

        const pose_t* pose_current = ctrl_get_pose_current(ctrl);

        if (trajectory_get_route_update(ctrl, pose_current, path)) {
            ctrl_set_mode(ctrl, CTRL_MODE_STOP);
        }
        else {
            ctrl_set_mode(ctrl, CTRL_MODE_RUNNING);
        }

        ctrl_set_speed_order(ctrl, speed_order);

yield_point:
        xtimer_periodic_wakeup(&loop_start_time, TASK_PERIOD_MS * US_PER_MS);
    }

    return 0;
}

void pln_init(void)
{
#ifdef MODULE_SHELL_PLANNERS
    pln_shell_init();
#endif /* MODULE_SHELL_PLANNERS */
}
