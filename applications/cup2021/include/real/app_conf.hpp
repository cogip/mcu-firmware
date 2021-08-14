#pragma once

/* Project includes */
#include "quadpid.h"

static const ctrl_quadpid_parameters_t ctrl_quadpid_params = {
    .linear_speed_pid = {
        .kp = 150.,
        .ki = 2,
        .kd = 0.,
    },
    .angular_speed_pid = {
        .kp = 150.,
        .ki = 2,
        .kd = 0.,
    },
    .linear_pose_pid = {
        .kp = 1,
        .ki = 0.,
        .kd = 2,
    },
    .angular_pose_pid = {
        .kp = 1,
        .ki = 0.,
        .kd = 5,
    },

    .min_distance_for_angular_switch = 3,       // mm,
    .min_angle_for_pose_reached = 2,            // deg,
    .regul = CTRL_REGUL_POSE_DIST,
};
