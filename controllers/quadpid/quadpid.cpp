/* Standard includes */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

/* RIOT includes */
#define ENABLE_DEBUG        (0)
#include "debug.h"
#include "irq.h"
#include "log.h"
#include "xtimer.h"

/* Project includes */
#include "ctrl.hpp"
#include "odometry.hpp"
#include "platform.hpp"
#include "quadpid.hpp"
#include "trigonometry.h"

/**
 * \fn cogip::cogip_defs::Polar compute_position_error(const cogip::cogip_defs::Pose &p1, const cogip::cogip_defs::Pose &p2)
 * \brief compute error between 2 poses
 * \param p1 : setpoint pose
 * \param p2 : measure pose
 * \return distance and angle errors between 2 poses
 */
static cogip::cogip_defs::Polar compute_position_error(const ctrl_quadpid_t *ctrl,
                                                       const cogip::cogip_defs::Pose &pose_order,
                                                       const cogip::cogip_defs::Pose &pose_current)
{
    cogip::cogip_defs::Polar error;
    double x, y, O;

    (void)ctrl;

    x = pose_order.x() - pose_current.x();
    y = pose_order.y() - pose_current.y();

    O = limit_angle_rad(atan2(y, x) - DEG2RAD(pose_current.O()));

    error.set_angle(RAD2DEG(O));
    error.set_distance(sqrt(square(x) + square(y)));

    return error;
}

/**
 * \fn limit_speed_command
 * \brief limit speed command to maximum acceleration and speed setpoint
 * \param command       computed speed by position PID controller
 * \param final_speed   maximum speed
 * \param real_speed    real speed
 * \param max_acc       maximum acceleration
 * \return speed_order
 */
static double limit_speed_command(double command,
                                  double final_speed,
                                  double real_speed,
                                  double max_acc)
{
    /* limit speed command (maximum acceleration) */
    double a = command - real_speed;

    if (a > max_acc) {
        command = real_speed + max_acc;
    }

    if (a < -max_acc) {
        command = real_speed - max_acc;
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

void ctrl_quadpid_set_pose_to_reach_cb(ctrl_t *ctrl) {
    ctrl_quadpid_t *ctrl_quadpid = (ctrl_quadpid_t *)ctrl;

    ctrl_quadpid->quadpid_params.regul = CTRL_REGUL_POSE_DIST;
}

int ctrl_quadpid_speed(ctrl_quadpid_t *ctrl,
                       cogip::cogip_defs::Polar &command,
                       const cogip::cogip_defs::Polar &speed_current)
{
    cogip::cogip_defs::Polar speed_error;

    DEBUG("@robot@,%u,%" PRIu32 ",@speed_order@,%.2f,%.2f\n",
          ROBOT_ID,
          ctrl->control.current_cycle,
          command.distance(),
          command.angle());

    speed_error.set_distance(command.distance() - speed_current.distance());
    speed_error.set_angle(command.angle() - speed_current.angle());

    if ((ctrl->control.anti_blocking_on)
        && (fabs(speed_current.distance()) <
            ctrl->pf_conf->blocking_speed_treshold)
        && (fabs(speed_error.distance()) >
            ctrl->pf_conf->blocking_speed_error_treshold)) {
        ctrl->control.blocking_cycles++;
    }
    else {
        ctrl->control.blocking_cycles = 0;
    }

    if (ctrl->control.blocking_cycles >= ctrl->pf_conf->blocking_cycles_max) {
        command.set_distance(0);
        command.set_angle(0);
        ctrl_set_mode((ctrl_t *)ctrl, CTRL_MODE_BLOCKED);
        ctrl->control.blocking_cycles = 0;

        return 0;
    }

    command.set_distance(pid_ctrl(&ctrl->quadpid_params.linear_speed_pid,
                                  speed_error.distance()));
    command.set_angle(pid_ctrl(&ctrl->quadpid_params.angular_speed_pid,
                               speed_error.angle()));

    return 0;
}

int ctrl_quadpid_stop(ctrl_t *ctrl, cogip::cogip_defs::Polar &command)
{
    (void)ctrl;

    pid_reset(&((ctrl_quadpid_t *)ctrl)->quadpid_params.linear_pose_pid);
    pid_reset(&((ctrl_quadpid_t *)ctrl)->quadpid_params.angular_pose_pid);
    pid_reset(&((ctrl_quadpid_t *)ctrl)->quadpid_params.linear_speed_pid);
    pid_reset(&((ctrl_quadpid_t *)ctrl)->quadpid_params.angular_speed_pid);

    command.set_distance(0);
    command.set_angle(0);

    return 0;
}

int ctrl_quadpid_nopid(ctrl_t *ctrl, cogip::cogip_defs::Polar &command)
{
    /* Get speed order */
    const cogip::cogip_defs::Polar &speed_order = ctrl_get_speed_order((ctrl_t *)ctrl);

    /* Compute speed order */
    ctrl_compute_speed_order((ctrl_t *)ctrl);

    command.set_distance(speed_order.distance());
    command.set_angle(speed_order.angle());

    return 0;
}

int ctrl_quadpid_running_speed(ctrl_t *ctrl, cogip::cogip_defs::Polar &command)
{
    /* Get speed current */
    const cogip::cogip_defs::Polar &speed_current = ctrl_get_speed_current(ctrl);
    /* Get speed order */
    const cogip::cogip_defs::Polar &speed_order = ctrl_get_speed_order((ctrl_t *)ctrl);

    /* Compute speed order */
    ctrl_compute_speed_order((ctrl_t *)ctrl);

    command.set_distance(speed_order.distance());
    command.set_angle(speed_order.angle());

    /* limit speed command->*/
    command.set_distance(limit_speed_command(command.distance(),
                                             fabs(speed_order.distance()),
                                             speed_current.distance(), MAX_ACC_LINEAR));
    command.set_angle(limit_speed_command(command.angle(),
                                          fabs(speed_order.angle()),
                                          speed_current.angle(), MAX_ACC_ANGULAR));

    /* ********************** speed pid controller ********************* */
    return ctrl_quadpid_speed((ctrl_quadpid_t *)ctrl, command, speed_current);
}

int ctrl_quadpid_ingame(ctrl_t *ctrl, cogip::cogip_defs::Polar &command)
{
    const cogip::cogip_defs::Pose &pose_current = ctrl_get_pose_current(ctrl);
    const cogip::cogip_defs::Polar &speed_current = ctrl_get_speed_current(ctrl);

    ctrl_quadpid_t *ctrl_quadpid = (ctrl_quadpid_t *)ctrl;
    /* get speed order */
    const cogip::cogip_defs::Polar &speed_order = ctrl_get_speed_order((ctrl_t *)ctrl_quadpid);

    /* ******************** position pid ctrl ******************** */

    /* get next pose to reach */
    cogip::cogip_defs::Pose pose_order = ctrl_get_pose_to_reach((ctrl_t *)ctrl_quadpid);

    /* compute position error */
    cogip::cogip_defs::Polar pos_err = compute_position_error(ctrl_quadpid, pose_order, pose_current);

    DEBUG("@robot@,%u,%" PRIu32 ",@pose_order@,%.2f,%.2f,%.2f\n",
          ROBOT_ID,
          ctrl->control.current_cycle,
          pose_order.x(),
          pose_order.y(),
          pose_order.O());

    /* position correction */
    if (ctrl_quadpid->quadpid_params.regul != CTRL_REGUL_POSE_ANGL
        && fabs(pos_err.distance()) > ctrl_quadpid->quadpid_params.min_distance_for_angular_switch) {

        /* should we go reverse? */
        if (ctrl_quadpid->control.allow_reverse && fabs(pos_err.angle()) > 90) {
            pos_err.set_distance(- pos_err.distance());

            if (pos_err.angle() < 0) {
                pos_err.set_angle(pos_err.angle() + 180);
            }
            else {
                pos_err.set_angle(pos_err.angle() - 180);
            }
        }

        /* if target point direction angle is too important, bot rotates on its starting point */
        if (fabs(pos_err.angle()) > ctrl_quadpid->quadpid_params.min_angle_for_target_orientation) {
            ctrl_quadpid->quadpid_params.regul = CTRL_REGUL_POSE_PRE_ANGL;
            pos_err.set_distance(0);
            pid_reset(&ctrl_quadpid->quadpid_params.linear_pose_pid);
            pid_reset(&ctrl_quadpid->quadpid_params.linear_speed_pid);
        }
        else {
            if (ctrl_quadpid->quadpid_params.regul == CTRL_REGUL_POSE_PRE_ANGL) {
                pid_reset(&ctrl_quadpid->quadpid_params.linear_pose_pid);
                pid_reset(&ctrl_quadpid->quadpid_params.angular_pose_pid);
                pid_reset(&ctrl_quadpid->quadpid_params.linear_speed_pid);
                pid_reset(&ctrl_quadpid->quadpid_params.angular_speed_pid);
            }
            ctrl_quadpid->quadpid_params.regul = CTRL_REGUL_POSE_DIST;
        }
    }
    else {
        /* orientation correction (position is reached) */
        ctrl_quadpid->quadpid_params.regul = CTRL_REGUL_POSE_ANGL;

        /* final orientation error */
        if (!ctrl_quadpid->control.pose_intermediate) {
            pos_err.set_angle(limit_angle_deg(pose_order.O() - pose_current.O()));
        }
        else {
            pos_err.set_angle(0);
        }

        pos_err.set_distance(0);
        pid_reset(&ctrl_quadpid->quadpid_params.linear_pose_pid);
        pid_reset(&ctrl_quadpid->quadpid_params.linear_speed_pid);

        /* orientation is reached */
        if (fabs(pos_err.angle()) < ctrl_quadpid->quadpid_params.min_angle_for_pose_reached) {
            pos_err.set_angle(0);
            pid_reset(&ctrl_quadpid->quadpid_params.linear_pose_pid);
            pid_reset(&ctrl_quadpid->quadpid_params.angular_pose_pid);
            pid_reset(&ctrl_quadpid->quadpid_params.linear_speed_pid);
            pid_reset(&ctrl_quadpid->quadpid_params.angular_speed_pid);

            ctrl_set_pose_reached(ctrl);

            command = {0, 0};
            return 0;
        }
    }

    /* compute speed command->with position pid controller */
    command.set_distance(pid_ctrl(&ctrl_quadpid->quadpid_params.linear_pose_pid,
                                  pos_err.distance()));
    command.set_angle(pid_ctrl(&ctrl_quadpid->quadpid_params.angular_pose_pid,
                               pos_err.angle()));

    DEBUG("@robot@,%u,%" PRIu32 ",@pose_set@,%.2f,%.2f\n",
          ROBOT_ID,
          ctrl->control.current_cycle,
          command.distance(),
          command.angle());

    /* limit speed command->*/
    command.set_distance(limit_speed_command(command.distance(),
                                             speed_order.distance(),
                                             speed_current.distance(), MAX_ACC_LINEAR));
    command.set_angle(limit_speed_command(command.angle(),
                                          speed_order.angle(),
                                          speed_current.angle(), MAX_ACC_ANGULAR));

    /* ********************** speed pid controller ********************* */
    return ctrl_quadpid_speed(ctrl_quadpid, command, speed_current);
}
