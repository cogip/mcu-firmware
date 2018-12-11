#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "irq.h"
#include "xtimer.h"
#include "system/log.h"
#include "odometry.h"
#include "platform.h"
#include "trigonometry.h"

#include "controller.h"

static void set_pose_reached(ctrl_t *ctrl)
{
    if (ctrl->pose_reached) {
        return;
    }

    ctrl->pose_reached = TRUE;
    printf("pose reached\n");
}

/**
 * \fn polar_t compute_position_error(const pose_t p1, const pose_t p2)
 * \brief compute error between 2 poses
 * \param p1 : setpoint pose
 * \param p2 : measure pose
 * \return distance and angle errors between 2 poses
 */
static polar_t compute_position_error(ctrl_t *ctrl,
                             const pose_t pose_order, const pose_t *pose_current)
{
    polar_t error;
    double x, y, O;

    (void)ctrl;

    x = pose_order.x - pose_current->x;
    y = pose_order.y - pose_current->y;

    O = limit_angle_rad(atan2(y, x) - DEG2RAD(pose_current->O));

    error.angle = RAD2DEG(O);
    error.distance = sqrt(square(x) + square(y));

    return error;
}

/**
 * \fn limit_speed_command
 * \brief limit speed command to maximum acceleration and speed setpoint
 * \param command : computed speed by position PID controller
 * \param final_speed : maximum speed
 * \param real_speed
 * \return speed_order
 */
static double limit_speed_command(double command,
                                  double final_speed,
                                  double real_speed)
{
    /* limit speed command (maximum acceleration) */
    double a = command - real_speed;

    if (a > MAX_ACC) {
        command = real_speed + MAX_ACC;
    }

    if (a < -MAX_ACC) {
        command = real_speed - MAX_ACC;
    }

    /* limit speed command (speed setpoint) */
    if (command > final_speed) {
        command = final_speed;
    }

    if (command < -final_speed) {
        command = -final_speed;
    }

    return command;
}

/**
 *
 */
polar_t speed_ctrl(ctrl_t *ctrl,
                         polar_t speed_order, polar_t speed_current)
{
    polar_t speed_error;
    polar_t command;
    static uint8_t error_blocking = 0;

    speed_error.distance = speed_order.distance - speed_current.distance;
    speed_error.angle = speed_order.angle - speed_current.angle;

    double d = fabs(ctrl->linear_speed_pid.previous_error) - fabs(speed_error.distance);
    if (d < 0) {
        error_blocking++;
    }
    else {
        error_blocking = 0;
    }

    if (error_blocking >= CTRL_BLOCKING_NB_ITERATIONS) {
        command.distance = 0;
        command.angle = 0;
        ctrl_set_mode(ctrl, CTRL_STATE_BLOCKED);
    }

    command.distance = pid_ctrl(&ctrl->linear_speed_pid,
                                      speed_error.distance);
    command.angle = pid_ctrl(&ctrl->angular_speed_pid,
                                   speed_error.angle);

    return command;
}

polar_t ctrl_update(ctrl_t *ctrl,
                          const pose_t *pose_current,
                          polar_t speed_current)
{
    polar_t command = { 0, 0 };
    pose_t pose_order = { 0, 0, 0 };
    polar_t speed_order = { 0, 0 };
    polar_t speed;
    /* ******************** position pid ctrl ******************** */

    /* compute position error */
    polar_t pos_err;

    if (ctrl_is_pose_reached(ctrl)) {
        return command;
    }

    /* get next pose_t to reach */
    pose_order = ctrl_get_pose_to_reach(ctrl);

    pose_order.x *= PULSE_PER_MM;
    pose_order.y *= PULSE_PER_MM;

    /* get speed order */
    speed_order = ctrl_get_speed_order(ctrl);

    pos_err = compute_position_error(ctrl, pose_order, pose_current);

    cons_printf("@c@,%+.0f,%+.0f,%+.0f,%+.0f,%+.0f,%+.0f,"
                "%+.0f,%+.0f,"
                "%+.0f,%+.0f,"
                "%+.0f,%+.0f,"
                "\n",
                pose_order.x / PULSE_PER_MM,
                pose_order.y / PULSE_PER_MM,
                pose_order.O,
                pose_current->x / PULSE_PER_MM,
                pose_current->y / PULSE_PER_MM,
                pose_current->O,
                pos_err.distance / PULSE_PER_MM,
                pos_err.angle,
                speed_order.distance / PULSE_PER_MM,
                speed_order.angle,
                speed_current.distance / PULSE_PER_MM,
                speed_current.angle);

    /* position correction */
    if (ctrl->regul != CTRL_REGUL_POSE_ANGL
        && fabs(pos_err.distance) > ctrl->min_distance_for_angular_switch) {

        /* should we go reverse? */
        if (ctrl->allow_reverse && fabs(pos_err.angle) > 90) {
            ctrl->in_reverse = TRUE;

            pos_err.distance = -pos_err.distance;

            if (pos_err.angle < 0) {
                pos_err.angle += 180;
            }
            else {
                pos_err.angle -= 180;
            }
        }
        else {
            ctrl->in_reverse = FALSE;
        }

        /* if target point direction angle is too important, bot rotates on its starting point */
        if (fabs(pos_err.angle) > ctrl->min_angle_for_pose_reached / PULSE_PER_DEGREE) {
            ctrl->regul = CTRL_REGUL_POSE_PRE_ANGL;
            pos_err.distance = 0;
            pid_reset(&ctrl->linear_pose_pid);
        }
        else {
            ctrl->regul = CTRL_REGUL_POSE_DIST;
        }
    }
    else {
        /* orientation correction (position is reached) */
        ctrl->regul = CTRL_REGUL_POSE_ANGL;

        /* final orientation error */
        if (!ctrl->pose_intermediate) {
            pos_err.angle = limit_angle_deg(pose_order.O - pose_current->O);
        }
        else {
            pos_err.angle = 0;
        }

        pos_err.distance = 0;
        pid_reset(&ctrl->linear_pose_pid);

        /* orientation is reached */
        if (fabs(pos_err.angle) < ctrl->min_angle_for_pose_reached / PULSE_PER_DEGREE) {
            pos_err.angle = 0;
            pid_reset(&ctrl->angular_pose_pid);

            set_pose_reached(ctrl);
            ctrl->regul = CTRL_REGUL_POSE_DIST; //CTRL_REGUL_IDLE;
        }
    }

    pos_err.angle *= PULSE_PER_DEGREE;

    /* compute speed command with position pid controller */
    command.distance = pid_ctrl(&ctrl->linear_pose_pid,
                                      pos_err.distance);
    command.angle = pid_ctrl(&ctrl->angular_pose_pid,
                                   pos_err.angle);


    /* limit speed command */
    speed.distance = limit_speed_command(command.distance,
                                         speed_order.distance,
                                         speed_current.distance);
    speed.angle = limit_speed_command(command.angle,
                                      speed_order.angle,
                                      speed_current.angle);

    /* ********************** speed pid controller ********************* */
    return speed_ctrl(ctrl, speed, speed_current);
}

inline void ctrl_set_pose_intermediate(ctrl_t *ctrl, uint8_t intermediate)
{
    ctrl->pose_intermediate = intermediate;
}

inline uint8_t ctrl_is_in_reverse(ctrl_t *ctrl)
{
    return ctrl->in_reverse;
}

inline void ctrl_set_allow_reverse(ctrl_t *ctrl, uint8_t allow)
{
    ctrl->allow_reverse = allow;
}

inline uint8_t ctrl_is_pose_reached(ctrl_t *ctrl)
{
    return ctrl->pose_reached;
}

inline void ctrl_set_pose_current(ctrl_t *ctrl, const pose_t pose)
{
    irq_disable();
    ctrl->pose_current = pose;
    irq_enable();
}

inline pose_t ctrl_get_pose_current(ctrl_t *ctrl)
{
    pose_t pose_current;

    irq_disable();
    pose_current = ctrl->pose_current;
    irq_enable();

    return pose_current;
}

inline void ctrl_set_pose_to_reach(ctrl_t *ctrl, const pose_t pose_order)
{
    irq_disable();
    if (!pose_equal(&ctrl->pose_order, &pose_order)) {
        ctrl->pose_order = pose_order;
        ctrl->pose_reached = FALSE;
    }
    irq_enable();
}

inline pose_t ctrl_get_pose_to_reach(ctrl_t *ctrl)
{
    pose_t pose_order;

    irq_disable();
    pose_order = ctrl->pose_order;
    irq_enable();

    return pose_order;
}

inline void ctrl_set_speed_order(ctrl_t *ctrl, const polar_t speed_order)
{
    irq_disable();
    ctrl->speed_order = speed_order;
    irq_enable();
}

inline polar_t ctrl_get_speed_order(ctrl_t *ctrl)
{
    polar_t speed_order;

    irq_disable();
    speed_order = ctrl->speed_order;
    irq_enable();

    return speed_order;
}

void ctrl_set_mode(ctrl_t *ctrl, ctrl_mode_id_t new_mode)
{
    ctrl->mode = &ctrl_modes[new_mode];
    printf("new_mode = %s\n", ctrl->mode->name);
}

void motor_drive(polar_t *command)
{
    /************************ commandes moteur ************************/
    int16_t right_command = (int16_t) (command->distance + command->angle);
    int16_t left_command = (int16_t) (command->distance - command->angle);

    motor_set(0, HBRIDGE_MOTOR_LEFT, left_command);
    motor_set(0, HBRIDGE_MOTOR_RIGHT, right_command);

    log_vect_setvalue(&datalog, LOG_IDX_MOTOR_L, (void *) &left_command);
    log_vect_setvalue(&datalog, LOG_IDX_MOTOR_R, (void *) &right_command);
}

void *task_ctrl_update(void *arg)
{
    /* bot position on the 'table' (absolute position): */
    polar_t motor_command = { 0, 0 };
    func_cb_t pfn_evtloop_prefunc = mach_get_ctrl_loop_pre_pfn();
    func_cb_t pfn_evtloop_postfunc = mach_get_ctrl_loop_post_pfn();

    ctrl_t *ctrl = (ctrl_t*)arg;
    printf("Controller started\n");

    for (;;) {
        xtimer_ticks32_t loop_start_time = xtimer_now();

        /* Machine specific stuff, if required */
        if (pfn_evtloop_prefunc) {
            (*pfn_evtloop_prefunc)();
        }

        if ((ctrl->mode) && (ctrl->mode->state_cb)) {
            ctrl->mode->state_cb(&ctrl->pose_current, &motor_command);
        }

        motor_drive(&motor_command);

        /* Machine specific stuff, if required */
        if (pfn_evtloop_postfunc) {
            (*pfn_evtloop_postfunc)();
        }

        xtimer_periodic_wakeup(&loop_start_time, THREAD_PERIOD_INTERVAL);
    }

    return 0;
}
