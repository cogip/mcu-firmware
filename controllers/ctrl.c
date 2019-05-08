/* RIOT includes */
#include "irq.h"
#include "log.h"
#include "xtimer.h"

/* Project includes */
#include "ctrl.h"
#include "utils.h"
#include "platform.h"

void ctrl_set_pose_reached(ctrl_t* ctrl)
{
    if (ctrl->control.pose_reached) {
        return;
    }

    ctrl->control.pose_reached = TRUE;
    printf("pose reached\n");
}

inline void ctrl_set_pose_intermediate(ctrl_t* ctrl, uint8_t intermediate)
{
    ctrl->control.pose_intermediate = intermediate;
}

inline void ctrl_set_allow_reverse(ctrl_t* ctrl, uint8_t allow)
{
    ctrl->control.allow_reverse = allow;
}

inline uint8_t ctrl_is_pose_reached(ctrl_t* ctrl)
{
    return ctrl->control.pose_reached;
}

inline void ctrl_set_pose_current(ctrl_t* const ctrl, const pose_t* pose_current)
{
    irq_disable();
    ctrl->control.pose_current = *pose_current;
    irq_enable();
}

inline const pose_t* ctrl_get_pose_current(ctrl_t* ctrl)
{
    return &ctrl->control.pose_current;
}

inline void ctrl_set_pose_to_reach(ctrl_t* ctrl, const pose_t* pose_order)
{
    irq_disable();
    if (!pose_equal(&ctrl->control.pose_order, pose_order)) {
        ctrl->control.pose_order = *pose_order;
        ctrl->control.pose_reached = FALSE;

        LOG_INFO("@robot@,@pose_order@,%u,%.2f,%.2f,%.2f\n",
                    ROBOT_ID,
                    pose_order->x,
                    pose_order->y,
                    pose_order->O);
    }
    irq_enable();
}

inline const pose_t* ctrl_get_pose_to_reach(ctrl_t* ctrl)
{
    return &ctrl->control.pose_order;
}

inline void ctrl_set_speed_current(ctrl_t* ctrl, const polar_t* speed_current)
{
    irq_disable();

    ctrl->control.speed_current = *speed_current;

    LOG_INFO("@robot@,@speed_current@,%u,%.2f,%.2f\n",
                ROBOT_ID,
                speed_current->distance,
                speed_current->angle);

    irq_enable();
}

inline const polar_t* ctrl_get_speed_current(ctrl_t* ctrl)
{
    return &ctrl->control.speed_current;
}

inline void ctrl_set_speed_order(ctrl_t* ctrl, polar_t* speed_order)
{
    irq_disable();

    ctrl->control.speed_order = speed_order;

    LOG_INFO("@robot@,@speed_order@,%u,%.2f,%.2f\n",
                ROBOT_ID,
                speed_order->distance,
                speed_order->angle);

    irq_enable();
}

inline polar_t* ctrl_get_speed_order(ctrl_t* ctrl)
{
    return ctrl->control.speed_order;
}

void ctrl_set_mode(ctrl_t* ctrl, ctrl_mode_t new_mode)
{
    if (new_mode < CTRL_MODE_NUMOF) {
        ctrl->control.current_mode = new_mode;
        LOG_DEBUG("ctrl: New mode: %d\n", ctrl->control.current_mode);
    }
    else {
        LOG_WARNING("ctrl: Unknown mode, stopping controller\n");
        ctrl->control.current_mode = CTRL_MODE_STOP;
    }
}

void *task_ctrl_update(void *arg)
{
    /* bot position on the 'table' (absolute position): */
    polar_t motor_command = { 0, 0 };

    ctrl_t *ctrl = (ctrl_t*)arg;
    LOG_DEBUG("ctrl: Controller started\n");

    for (;;) {
        xtimer_ticks32_t loop_start_time = xtimer_now();

        ctrl_mode_t current_mode = ctrl->control.current_mode;

        ctrl_pre_mode_cb_t pre_mode_cb = ctrl->pf_conf->ctrl_pre_mode_cb[ctrl->control.current_mode];

        if (ctrl->pf_conf->ctrl_pre_mode_cb[ctrl->control.current_mode]) {
            pre_mode_cb(&ctrl->control.pose_current, &ctrl->control.speed_current, &motor_command);
        }

        ctrl_mode_cb_t mode_cb = ctrl->conf->ctrl_mode_cb[ctrl->control.current_mode];

        if (ctrl->conf->ctrl_mode_cb[current_mode]) {
            mode_cb(ctrl, &motor_command);
        }

        ctrl_post_mode_cb_t post_mode_cb = ctrl->pf_conf->ctrl_post_mode_cb[ctrl->control.current_mode];

        if (ctrl->pf_conf->ctrl_post_mode_cb[current_mode]) {
            post_mode_cb(&ctrl->control.pose_current, &ctrl->control.speed_current, &motor_command);
        }

        xtimer_periodic_wakeup(&loop_start_time, THREAD_PERIOD_INTERVAL);
    }

    return 0;
}
