/* RIOT includes */
#define ENABLE_DEBUG        (0)
#include "debug.h"
#include "irq.h"
#include "log.h"
#include "xtimer.h"

/* Project includes */
#include "ctrl.hpp"
#include "utils.h"
#include "platform.hpp"

void ctrl_set_pose_reached(ctrl_t *ctrl)
{
    if (ctrl->control.pose_reached) {
        return;
    }

    DEBUG("ctrl: Pose is reached\n");

    ctrl->control.pose_reached = TRUE;
}

uint32_t ctrl_get_current_cycle(ctrl_t *ctrl)
{
    return ctrl->control.current_cycle;
}

void ctrl_set_pose_intermediate(ctrl_t *ctrl, uint8_t intermediate)
{
    if (intermediate) {
        DEBUG("ctrl: Next pose is intermediate\n");
    }

    ctrl->control.pose_intermediate = intermediate;
}

uint8_t ctrl_is_pose_intermediate(ctrl_t *ctrl)
{
    return ctrl->control.pose_intermediate;
}

void ctrl_set_allow_reverse(ctrl_t *ctrl, uint8_t allow)
{
    ctrl->control.allow_reverse = allow;
}

void ctrl_set_anti_blocking_on(ctrl_t *ctrl, uint8_t value)
{
    ctrl->control.anti_blocking_on = value;
}

uint8_t ctrl_get_anti_blocking_on(ctrl_t *ctrl)
{
    return ctrl->control.anti_blocking_on;
}

uint8_t ctrl_is_pose_reached(ctrl_t *ctrl)
{
    return ctrl->control.pose_reached;
}

void ctrl_set_pose_current(ctrl_t *const ctrl, const cogip::cogip_defs::Pose &pose_current)
{
    DEBUG("ctrl: New pose current: x=%lf, y=%lf, O=%lf\n",
          pose_current.x(), pose_current.y(), pose_current.O());

    irq_disable();
    ctrl->control.pose_current = pose_current;
    irq_enable();
}

const cogip::cogip_defs::Pose &ctrl_get_pose_current(ctrl_t *ctrl)
{
    return ctrl->control.pose_current;
}

void ctrl_set_pose_to_reach(ctrl_t *ctrl, const cogip::cogip_defs::Pose &pose_order)
{
    DEBUG("ctrl: New pose to reach: x=%lf, y=%lf, O=%lf\n",
          pose_order.x(), pose_order.y(), pose_order.O());

    irq_disable();

    if (!(ctrl->control.pose_order == pose_order)) {
        ctrl->control.blocking_cycles = 0;
        ctrl->control.pose_order = pose_order;
        ctrl->control.pose_reached = FALSE;
    }

    irq_enable();
}

cogip::cogip_defs::Pose ctrl_get_pose_to_reach(ctrl_t *ctrl)
{
    return ctrl->control.pose_order;
}

const polar_t *ctrl_get_speed_current(ctrl_t *ctrl)
{
    return &ctrl->control.speed_current;
}

void ctrl_set_speed_order(ctrl_t *ctrl, polar_t speed_order)
{
    DEBUG("ctrl: New speed order: linear=%lf, angle=%lf\n",
          speed_order.distance, speed_order.angle);

    irq_disable();

    ctrl->control.speed_order = speed_order;

    irq_enable();
}

const polar_t *ctrl_get_speed_order(ctrl_t *ctrl)
{
    return &ctrl->control.speed_order;
}

void ctrl_register_speed_order_cb(ctrl_t *ctrl, speed_order_cb_t speed_order_cb)
{
    irq_disable();

    ctrl->control.speed_order_cb = speed_order_cb;
    ctrl->control.current_cycle = 0;

    irq_enable();
}

void ctrl_compute_speed_order(ctrl_t *ctrl)
{
    speed_order_cb_t cb = ctrl->control.speed_order_cb;

    if (cb) {
        /* Variable speed_order is computable through a callback */
        ctrl->control.speed_order = (*cb)(ctrl);
    }
}

void ctrl_set_mode(ctrl_t *ctrl, ctrl_mode_t new_mode)
{
    /* Ensure we don't set a non existant mode */
    if (new_mode >= CTRL_MODE_NUMOF) {
        LOG_WARNING("ctrl: Unknown mode, stopping controller\n");
        ctrl->control.current_mode = CTRL_MODE_STOP;
        return;
    }

    irq_disable();

    if (new_mode != ctrl->control.current_mode) {
        ctrl->control.current_mode = new_mode;

#if ENABLE_DEBUG == 1
        printf("ctrl: New mode: ");
        switch (new_mode) {
            case CTRL_MODE_STOP:
                puts("CTRL_MODE_STOP"); break;
            case CTRL_MODE_IDLE:
                puts("CTRL_MODE_IDLE"); break;
            case CTRL_MODE_BLOCKED:
                puts("CTRL_MODE_BLOCKED"); break;
            case CTRL_MODE_RUNNING:
                puts("CTRL_MODE_RUNNING"); break;
            case CTRL_MODE_RUNNING_SPEED:
                puts("CTRL_MODE_RUNNING_SPEED"); break;
            case CTRL_MODE_PASSTHROUGH:
                puts("CTRL_MODE_PASSTHROUGH"); break;
            default:
                puts("<unknown>"); break;
        }
#endif

        /* Reset current cycle as current mode has changed */
        ctrl->control.current_cycle = 0;
    }

    irq_enable();
}

ctrl_mode_t ctrl_get_mode(ctrl_t *ctrl)
{
    return ctrl->control.current_mode;
}

void *task_ctrl_update(void *arg)
{
    /* bot position on the 'table' (absolute position): */
    polar_t motor_command = { 0, 0 };

    ctrl_t *ctrl = (ctrl_t *)arg;

    DEBUG("ctrl: Controller started\n");

    for (;;) {
        xtimer_ticks32_t loop_start_time = xtimer_now();

        ctrl_mode_t current_mode = ctrl->control.current_mode;

        ctrl_pre_mode_cb_t pre_mode_cb = ctrl->pf_conf->ctrl_pre_mode_cb[current_mode];

        if (pre_mode_cb) {
            pre_mode_cb(ctrl->control.pose_current, &ctrl->control.speed_current, &motor_command);
        }

        ctrl_mode_cb_t mode_cb = ctrl->conf->ctrl_mode_cb[current_mode];

        if (mode_cb) {
            mode_cb(ctrl, &motor_command);
        }

        ctrl_post_mode_cb_t post_mode_cb = ctrl->pf_conf->ctrl_post_mode_cb[current_mode];

        if (post_mode_cb) {
            post_mode_cb(ctrl->control.pose_current, &ctrl->control.speed_current, &motor_command);
        }

        /* Current cycle finished */
        ctrl->control.current_cycle++;

        xtimer_periodic_wakeup(&loop_start_time, THREAD_PERIOD_INTERVAL);
    }

    return 0;
}
