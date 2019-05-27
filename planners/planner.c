#include "planner.h"

#include <stdio.h>
#include "avoidance.h"
#include "ctrl.h"
#include "xtimer.h"
#include "platform.h"
#include "trigonometry.h"
#include "obstacle.h"
#include <irq.h>
#include <periph/adc.h>

/* RIOT includes */
#define ENABLE_DEBUG        (0)
#include "debug.h"
#include "log.h"

static uint8_t pln_started = FALSE;

/* Planner can automatically change next path pose to reach when current pose
 * is reached */
static uint8_t allow_change_path_pose = TRUE;

/* Periodic task */
#define TASK_PERIOD_MS      (200)

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

static int trajectory_get_route_update(ctrl_t* ctrl, const pose_t *robot_pose,
        pose_t *pose_to_reach, polar_t *speed_order, path_t *path)
{
    const path_pose_t *current_path_pos = path_get_current_path_pos(path);
    static uint8_t index = 1;
    int test = 0;
    int control_loop = 0;
    uint8_t need_update = 0;

    need_update = pf_read_sensors();

    if (ctrl_is_pose_reached(ctrl)) {
        if ((pose_to_reach->x == current_path_pos->pos.x)
            && (pose_to_reach->y == current_path_pos->pos.y)) {
            DEBUG("planner: Controller has reach final position.\n");
            if (allow_change_path_pose) {
                if (current_path_pos->act)
                    (*(current_path_pos->act))();
                path_increment_current_pose_idx(path);
            }
            current_path_pos = path_get_current_path_pos(path);
            need_update = 1;
        }
        else if ((!allow_change_path_pose) && (!ctrl_get_pose_intermediate(ctrl))) {
            current_path_pos = path_get_current_path_pos(path);
            need_update = 1;
        }
        else {
            DEBUG("planner: Controller has reach intermediate position.\n");
            index++;
        }
    }

    if (ctrl->control.current_mode == CTRL_MODE_BLOCKED) {
        DEBUG("planner: Controller is blocked.\n");
        if (!allow_change_path_pose)
            goto trajectory_get_route_update_error;
        path_increment_current_pose_idx(path);
        current_path_pos = path_get_current_path_pos(path);
        need_update = 1;
    }

    if (need_update) {
        test = update_graph(robot_pose, &(current_path_pos->pos));

        control_loop = path->nb_pose;
        while ((test < 0) && (control_loop-- > 0)) {
            if (test == -1) {
                if (!allow_change_path_pose)
                    goto trajectory_get_route_update_error;
                path_increment_current_pose_idx(path);
                if (current_path_pos == path_get_current_path_pos(path))
                    goto trajectory_get_route_update_error;
                current_path_pos = path_get_current_path_pos(path);
            }
            test = update_graph(robot_pose, &(current_path_pos->pos));
        }
        if (control_loop < 0) {
            LOG_ERROR("planner: No position reachable !\n");
            goto trajectory_get_route_update_error;
        }
        index = 1;
    }

    *pose_to_reach = avoidance(index);
    if ((pose_to_reach->x == current_path_pos->pos.x)
        && (pose_to_reach->y == current_path_pos->pos.y)) {
        pose_to_reach->O = current_path_pos->pos.O;
        ctrl_set_pose_intermediate(ctrl, FALSE);
        DEBUG("planner: Reaching final position\n");
    }
    else {
        /* Update speed order to max speed defined value in the new point to reach */
        speed_order->distance = path_get_current_max_speed(path);
        speed_order->angle = speed_order->distance / 2;
        ctrl_set_pose_intermediate(ctrl, TRUE);
        DEBUG("planner: Reaching intermediate position\n");
    }

    return 0;

trajectory_get_route_update_error:
    return -1;
}

void *task_planner(void *arg)
{
    (void)arg;
    pose_t pose_order = { 0, 0, 0 };
    pose_t initial_pose = { 0, 0, 0 };
    polar_t speed_order = { 0, 0 };
    polar_t initial_speed = { 0, 0 };
    const uint8_t camp_left = pf_is_camp_left();
    const path_pose_t *current_path_pos = NULL;

    ctrl_t *ctrl = pf_get_ctrl();

    printf("Game planner started\n");
    /* 2019: Camp left is purple, right is yellow */
    printf("%s camp\n", camp_left ? "LEFT" : "RIGHT");

    path_t* path = pf_get_path();
    if (!path) {
        printf("machine has no path\n");
    }

    /* mirror the points in place if selected camp is left */
    if (camp_left) {
        path_horizontal_mirror_all_pos(path);
    }

    /* object context initialisation */
    path->current_pose_idx = 0;
    current_path_pos = path_get_current_path_pos(path);
    initial_pose = current_path_pos->pos;
    pose_order = current_path_pos->pos;
    ctrl_set_pose_current(ctrl, &initial_pose);
    ctrl_set_speed_current(ctrl, &initial_speed);
    ctrl_set_pose_to_reach(ctrl, &pose_order);
    ctrl_set_pose_reached(ctrl);

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

        if (trajectory_get_route_update(ctrl, pose_current, &pose_order, &speed_order, path) == -1) {
            ctrl_set_mode(ctrl, CTRL_MODE_STOP);
        }
        else {
            ctrl_set_mode(ctrl, CTRL_MODE_RUNNING);
        }

        ctrl_set_speed_order(ctrl, &speed_order);

        ctrl_set_pose_to_reach(ctrl, &pose_order);

yield_point:
        xtimer_periodic_wakeup(&loop_start_time, TASK_PERIOD_MS * US_PER_MS);
    }

    return 0;
}
