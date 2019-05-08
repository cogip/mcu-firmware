/* Standard includes */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

/* RIOT includes */
#include "irq.h"
#include "log.h"
#include "xtimer.h"

/* Project includes */
#include "ctrl.h"
#include "odometry.h"
#include "platform.h"
#include "ctrl/quadpid.h"
#include "trigonometry.h"

/**
 * \fn polar_t compute_position_error(const pose_t p1, const pose_t p2)
 * \brief compute error between 2 poses
 * \param p1 : setpoint pose
 * \param p2 : measure pose
 * \return distance and angle errors between 2 poses
 */
static polar_t compute_position_error(const ctrl_quadpid_t* ctrl,
                             const pose_t *pose_order, const pose_t *pose_current)
{
    polar_t error;
    double x, y, O;

    (void)ctrl;

    x = pose_order->x - pose_current->x;
    y = pose_order->y - pose_current->y;

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
int ctrl_quadpid_speed(ctrl_quadpid_t* ctrl,
                         polar_t* command, const polar_t* speed_current)
{
    polar_t speed_error;
    static uint8_t error_blocking = 0;

    speed_error.distance = command->distance - speed_current->distance;
    speed_error.angle = command->angle - speed_current->angle;

    double d = fabs(ctrl->quadpid_params.linear_speed_pid.previous_error) - fabs(speed_error.distance);
    if (d < 0) {
        error_blocking++;
    }
    else {
        error_blocking = 0;
    }

    if (error_blocking >= CTRL_BLOCKING_NB_ITERATIONS) {
        command->distance = 0;
        command->angle = 0;
        ctrl_set_mode((ctrl_t*)ctrl, CTRL_MODE_BLOCKED);
    }

    command->distance = pid_ctrl(&ctrl->quadpid_params.linear_speed_pid,
                                      speed_error.distance);
    command->angle = pid_ctrl(&ctrl->quadpid_params.angular_speed_pid,
                                   speed_error.angle);

    return 0;
}

int ctrl_quadpid_stop(ctrl_t* ctrl, polar_t* command)
{
    (void)ctrl;

    if (!command) {
        LOG_ERROR("ctrl_quadpid: 'command' is NULL\n");
        return -1;
    }
    else {
        command->distance = 0;
        command->angle = 0;

        return 0;
    }
}

int ctrl_quadpid_running_speed(ctrl_t* ctrl, polar_t* command)
{
    polar_t* speed_order = NULL;

    const pose_t* pose_current = ctrl_get_pose_current(ctrl);
    const polar_t* speed_current = ctrl_get_speed_current(ctrl);

    ctrl_quadpid_t* ctrl_quadpid = (ctrl_quadpid_t*)ctrl;

    /* get speed order */
    speed_order = ctrl_get_speed_order((ctrl_t*)ctrl_quadpid);

    if((!speed_order)
        || (!command)
        || (ctrl_is_pose_reached((ctrl_t*)ctrl_quadpid))) {
        return -1;
    }

    LOG_INFO("@robot@,@pose_current@,%u,%.4f,%.4f,%.4f\n",
                ROBOT_ID,
                pose_current->x,
                pose_current->y,
                pose_current->O);

    LOG_INFO("@robot@,@speed_current@,%u,%.4f,%.4f\n",
                ROBOT_ID,
                speed_current->distance,
                speed_current->angle);


    command->distance = speed_order->distance;
    command->angle = speed_order->angle;

    /* limit speed command->*/
    command->distance = limit_speed_command(command->distance,
                                         fabs(speed_order->distance),
                                         speed_current->distance);
    command->angle = limit_speed_command(command->angle,
                                      fabs(speed_order->angle),
                                      speed_current->angle);

    /* ********************** speed pid controller ********************* */
    return ctrl_quadpid_speed(ctrl_quadpid, command, speed_current);
}

int ctrl_quadpid_ingame(ctrl_t* ctrl, polar_t* command)
{
    const pose_t* pose_order = NULL;
    polar_t* speed_order = NULL;

    const pose_t* pose_current = ctrl_get_pose_current(ctrl);
    const polar_t* speed_current = ctrl_get_speed_current(ctrl);

    ctrl_quadpid_t* ctrl_quadpid = (ctrl_quadpid_t*)ctrl;

    /* ******************** position pid ctrl ******************** */

    /* compute position error */
    polar_t pos_err;

    /* get next pose_t to reach */
    pose_order = ctrl_get_pose_to_reach((ctrl_t*)ctrl_quadpid);

    /* get speed order */
    speed_order = ctrl_get_speed_order((ctrl_t*)ctrl_quadpid);

    if ((!pose_order)
        || (!speed_order)
        || (!command)
        || (ctrl_is_pose_reached((ctrl_t*)ctrl_quadpid))) {
        return -1;
    }

    LOG_INFO("@robot@,@pose_current@,%u,%.0f,%.0f,%.0f\n",
                ROBOT_ID,
                pose_current->x,
                pose_current->y,
                pose_current->O);

    pos_err = compute_position_error(ctrl_quadpid, pose_order, pose_current);

    LOG_INFO("@robot@,@pose_error@,%u,%.0f,%.0f\n",
                ROBOT_ID,
                pos_err.distance,
                pos_err.angle);

    /* position correction */
    if (ctrl_quadpid->quadpid_params.regul != CTRL_REGUL_POSE_ANGL
        && fabs(pos_err.distance) > ctrl_quadpid->quadpid_params.min_distance_for_angular_switch) {

        /* should we go reverse? */
        if (ctrl_quadpid->control.allow_reverse && fabs(pos_err.angle) > 90) {
            pos_err.distance = -pos_err.distance;

            if (pos_err.angle < 0) {
                pos_err.angle += 180;
            }
            else {
                pos_err.angle -= 180;
            }
        }

        /* if target point direction angle is too important, bot rotates on its starting point */
        if (fabs(pos_err.angle) > ctrl_quadpid->quadpid_params.min_angle_for_pose_reached) {
            ctrl_quadpid->quadpid_params.regul = CTRL_REGUL_POSE_PRE_ANGL;
            pos_err.distance = 0;
            pid_reset(&ctrl_quadpid->quadpid_params.linear_pose_pid);
        }
        else {
            ctrl_quadpid->quadpid_params.regul = CTRL_REGUL_POSE_DIST;
        }
    }
    else {
        /* orientation correction (position is reached) */
        ctrl_quadpid->quadpid_params.regul = CTRL_REGUL_POSE_ANGL;

        /* final orientation error */
        if (!ctrl_quadpid->control.pose_intermediate) {
            pos_err.angle = limit_angle_deg(pose_order->O - pose_current->O);
        }
        else {
            pos_err.angle = 0;
        }

        pos_err.distance = 0;
        pid_reset(&ctrl_quadpid->quadpid_params.linear_pose_pid);

        /* orientation is reached */
        if (fabs(pos_err.angle) < ctrl_quadpid->quadpid_params.min_angle_for_pose_reached) {
            pos_err.angle = 0;
            pid_reset(&ctrl_quadpid->quadpid_params.angular_pose_pid);

            ctrl_set_pose_reached((ctrl_t*) ctrl_quadpid);
            ctrl_quadpid->quadpid_params.regul = CTRL_REGUL_POSE_DIST; //CTRL_REGUL_IDLE;
        }
    }

    /* compute speed command->with position pid controller */
    command->distance = pid_ctrl(&ctrl_quadpid->quadpid_params.linear_pose_pid,
                                      pos_err.distance);
    command->angle = pid_ctrl(&ctrl_quadpid->quadpid_params.angular_pose_pid,
                                   pos_err.angle);


    /* limit speed command->*/
    command->distance = limit_speed_command(command->distance,
                                         speed_order->distance,
                                         speed_current->distance);
    command->angle = limit_speed_command(command->angle,
                                      speed_order->angle,
                                      speed_current->angle);

    /* ********************** speed pid controller ********************* */
    return ctrl_quadpid_speed(ctrl_quadpid, command, speed_current);
}
