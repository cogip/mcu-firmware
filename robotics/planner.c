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
#include <log.h>

/* Object variables (singleton) */
static uint16_t game_time = 0;
static uint8_t game_started = FALSE;
path_t *path = NULL;

/* periodic task */
/* sched period = 20ms -> ticks freq is 1/0.02 = 50 Hz */
#define TASK_PERIOD_MS      (200)
#define TASK_FREQ_HZ        (1000 / TASK_PERIOD_MS)
#define GAME_DURATION_SEC   100
#define GAME_DURATION_TICKS (GAME_DURATION_SEC * TASK_FREQ_HZ)

static void show_game_time(void)
{
    static uint8_t _secs = TASK_FREQ_HZ;

    if (!--_secs) {
        _secs = TASK_FREQ_HZ;
        printf("Game time = %d\n",
               game_time / TASK_FREQ_HZ);
    }
}

void planner_start_game(ctrl_t* ctrl)
{
    /* TODO: send pose_initial, pose_order & speed_order to controller */
    ctrl_set_mode(ctrl, CTRL_STATE_INGAME);
    game_started = TRUE;
}

static int trajectory_get_route_update(ctrl_t* ctrl, const pose_t *robot_pose, pose_t *pose_to_reach, polar_t *speed_order)
{
    static pose_t pose_reached;
    const path_pose_t *current_path_pos = path_get_current_path_pos(path);
    pose_t robot_pose_tmp;
    static uint8_t index = 1;
    static int first_boot = 0;
    int test = 0;
    int control_loop = 0;
    uint8_t need_update = 0;

    (void) robot_pose;
    robot_pose_tmp =  *robot_pose;

    if (first_boot == 0) {
        first_boot = 1;
        pose_reached = current_path_pos->pos;
        *pose_to_reach = current_path_pos->pos;
        set_start_position_finish_position(&pose_reached, pose_to_reach);
        if (update_graph() == -1) {
            LOG_ERROR("planner: Update graph failed !\n");
            goto trajectory_get_route_update_error;
        }
    }

    if (ctrl_is_pose_reached(ctrl)) {
        pose_reached = *pose_to_reach;

        if ((pose_to_reach->x == current_path_pos->pos.x)
            && (pose_to_reach->y == current_path_pos->pos.y)) {
            LOG_DEBUG("planner: Controller has reach final position.\n");
            path_increment_current_pose_idx(path);
            current_path_pos = path_get_current_path_pos(path);
            robot_pose_tmp = pose_reached;
            need_update = 1;
        }
        else {
            LOG_DEBUG("planner: Controller has reach intermediate position.\n");
            index++;
        }
    }

    reset_dyn_polygons();

    if (ctrl->common.current_mode->mode_id == CTRL_STATE_BLOCKED) {
        path_increment_current_pose_idx(path);
        ctrl_set_mode(ctrl, CTRL_STATE_INGAME);
        need_update = 1;
    }

    if (need_update) {
        set_start_position_finish_position(&robot_pose_tmp, &(current_path_pos->pos));
        test = update_graph();

        control_loop = path->nb_pose;
        while ((test < 0) && (control_loop-- > 0)) {
            if (test == -1) {
                path_increment_current_pose_idx(path);
                current_path_pos = path_get_current_path_pos(path);
            }
            set_start_position_finish_position(&robot_pose_tmp, &(current_path_pos->pos));
            test = update_graph();
        }
        if (control_loop == 0) {
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
        LOG_DEBUG("planner: Reaching final position\n");
    }
    else {
        /* Update speed order to max speed defined value in the new point to reach */
        speed_order->distance = path_get_current_max_speed(path);
        speed_order->angle = speed_order->distance / 2;
        ctrl_set_pose_intermediate(ctrl, TRUE);
        LOG_DEBUG("planner: Reaching intermediate position\n");
    }

    return 0;

trajectory_get_route_update_error:
    return -1;
}

void *task_planner(void *arg)
{
    func_cb_t pfn_evtloop_end_of_game = pf_get_end_of_game_pfn();
    pose_t pose_order = { 0, 0, 0 };
    pose_t initial_pose = { 0, 0, 0 };
    polar_t speed_order = { 0, 0 };
    const uint8_t camp_left = pf_is_camp_left();
    const path_pose_t *current_path_pos = NULL;

    ctrl_t *ctrl = (ctrl_t*)arg;

    printf("Game planner started\n");
    /* 2018: Camp left is orange, right is green */
    printf("%s camp\n", camp_left ? "LEFT" : "RIGHT");

    path = pf_get_path();
    if (!path) {
        printf("machine has no path\n");
    }

    /* mirror the points in place if selected camp is right */
    if (!camp_left) {
        path_horizontal_mirror_all_pos(path);
    }

    /* object context initialisation */
    path->current_pose_idx = 0;
    current_path_pos = path_get_current_path_pos(path);
    initial_pose = current_path_pos->pos;
    ctrl_set_pose_current(ctrl, &initial_pose);

    uint32_t game_start_time = xtimer_now_usec();

    for (;;) {
        xtimer_ticks32_t loop_start_time = xtimer_now();

        if (!game_started) {
            goto yield_point;
        }

        if (pfn_evtloop_end_of_game && game_time >= GAME_DURATION_TICKS) {
            (*pfn_evtloop_end_of_game)();
        }

        /* while starter switch is not release we wait */
        if (!pf_is_game_launched()) {
            game_start_time = xtimer_now_usec();
            goto yield_point;
        }

        if (xtimer_now_usec() - game_start_time >= GAME_DURATION_SEC * US_PER_SEC) {
            cons_printf(">>>>\n");
            ctrl_set_mode(ctrl, CTRL_STATE_STOP);
            break;
        }

        if (!game_time) {
            cons_printf("<<<< polar_simu.csv\n");
            cons_printf("@command@,pose_order_x,pose_order_y,pose_order_a,"
                        "pose_current_x,pose_current_y,pose_current_a,"
                        "position_error_l,position_error_a,"
                        "speed_order_l,speed_order_a,"
                        "speed_current_l,speed_current_a,"
                        "game_time,"
                        "\n");
        }

        game_time++;
        show_game_time();

        /* ===== speed ===== */
        current_path_pos = path_get_current_path_pos(path);

        /* Update speed order to max speed defined value in the new point to reach */
        speed_order.distance = path_get_current_max_speed(path);
        speed_order.angle = speed_order.distance / 2;

        /* reverse gear selection is granted per point to reach, in path */
        ctrl_set_allow_reverse(ctrl, current_path_pos->allow_reverse);

        pose_t* pose_current = ctrl_get_pose_current(ctrl);

        /* ===== position ===== */
        if (trajectory_get_route_update(ctrl, pose_current, &pose_order, &speed_order) == -1) {
            ctrl_set_mode(ctrl, CTRL_STATE_STOP);
        }

        ctrl_set_speed_order(ctrl, &speed_order);

        ctrl_set_pose_to_reach(ctrl, &pose_order);

yield_point:
        xtimer_periodic_wakeup(&loop_start_time, TASK_PERIOD_MS * US_PER_MS);
    }

    return 0;
}
