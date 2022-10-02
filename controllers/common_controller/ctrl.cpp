/* RIOT includes */
#define ENABLE_DEBUG        (0)
#include "debug.h"
#include "irq.h"
#include "log.h"
#include "ztimer.h"

/* Project includes */
#include "ctrl.hpp"
#include "utils.hpp"
#include "platform.hpp"

#define TASK_PERIOD_USEC    (CONTROLLER_SPEED_LOOP_PERIOD_MSEC * US_PER_MS)

#ifdef MODULE_UARTPB
cogip::uartpb::UartProtobuf *uart_protobuf = nullptr;

void ctrl_register_uartpb(cogip::uartpb::UartProtobuf *uartpb_ptr)
{
    uart_protobuf = uartpb_ptr;
}
#endif // MODULE_UARTPB

void ctrl_set_pose_reached(ctrl_t *ctrl)
{
    if (ctrl->control.pose_reached) {
        return;
    }

    DEBUG("ctrl: Pose is reached\n");

    ctrl->control.pose_reached = TRUE;

#ifdef MODULE_UARTPB
    if (uart_protobuf) {
        uart_protobuf->send_message(pose_reached_uuid);
    }
#endif // MODULE_UARTPB
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

void ctrl_set_pose_to_reach(ctrl_t *ctrl, cogip::cogip_defs::Pose pose_order)
{
    DEBUG("ctrl: New pose to reach: x=%lf, y=%lf, O=%lf\n",
          pose_order.x(), pose_order.y(), pose_order.O());

    irq_disable();

    if (!(ctrl->control.pose_order == pose_order)) {
        ctrl->control.blocking_cycles = 0;
        ctrl->control.pose_order = pose_order;
        ctrl->control.pose_reached = FALSE;
        ctrl_set_pose_to_reach_cb_t set_pose_to_reach_cb = ctrl->conf->set_pose_to_reach_cb;
        if (set_pose_to_reach_cb) {
            set_pose_to_reach_cb(ctrl);
        }
    }

    irq_enable();
}

cogip::cogip_defs::Pose ctrl_get_pose_to_reach(ctrl_t *ctrl)
{
    return ctrl->control.pose_order;
}

const cogip::cogip_defs::Polar &ctrl_get_speed_current(ctrl_t *ctrl)
{
    return ctrl->control.speed_current;
}

void ctrl_set_speed_order(ctrl_t *ctrl, const cogip::cogip_defs::Polar &speed_order)
{
    DEBUG("ctrl: New speed order: linear=%lf, angle=%lf\n",
          speed_order.distance(), speed_order.angle());

    irq_disable();

    ctrl->control.speed_order = speed_order;

    irq_enable();
}

const cogip::cogip_defs::Polar &ctrl_get_speed_order(ctrl_t *ctrl)
{
    return ctrl->control.speed_order;
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
    cogip::cogip_defs::Polar motor_command;

    ctrl_t *ctrl = (ctrl_t *)arg;

    DEBUG("ctrl: Controller started\n");

    // Init loop iteration start time
    ztimer_now_t loop_start_time = ztimer_now(ZTIMER_USEC);

    for (;;) {

        ctrl_mode_t current_mode = ctrl->control.current_mode;

        ctrl_pre_mode_cb_t pre_mode_cb = ctrl->pf_conf->ctrl_pre_mode_cb[current_mode];

        if (pre_mode_cb) {
            pre_mode_cb(ctrl->control.pose_current, ctrl->control.speed_current, motor_command);
        }

        ctrl_mode_cb_t mode_cb = ctrl->conf->ctrl_mode_cb[current_mode];

        if (mode_cb) {
            mode_cb(ctrl, motor_command);
        }

        ctrl_post_mode_cb_t post_mode_cb = ctrl->pf_conf->ctrl_post_mode_cb[current_mode];

        if (post_mode_cb) {
            post_mode_cb(ctrl->control.pose_current, ctrl->control.speed_current, motor_command);
        }

        /* Current cycle finished */
        ctrl->control.current_cycle++;

        // Wait thread period to end
        ztimer_periodic_wakeup(ZTIMER_USEC, &loop_start_time, TASK_PERIOD_USEC);
    }

    return 0;
}
