#pragma once

/* Project includes */
#include "quadpid.h"

static const ctrl_quadpid_parameters_t ctrl_quadpid_params = {
    .linear_speed_pid = {
        .kp = 110.,
        .ki = 50,
        .kd = 0.,
    },
    .angular_speed_pid = {
        .kp = 110.,
        .ki = 50,
        .kd = 0.,
    },
    .linear_pose_pid = {
        .kp = 0.05,
        .ki = 0.,
        .kd = 0,
    },
    .angular_pose_pid = {
        .kp = 0.05,
        .ki = 0.,
        .kd = 0,
    },

    .min_distance_for_angular_switch = 2,       // mm,
    .min_angle_for_pose_reached = 2,            // deg,
    .regul = CTRL_REGUL_POSE_DIST,
};
