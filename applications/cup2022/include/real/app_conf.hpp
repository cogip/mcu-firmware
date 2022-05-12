#pragma once

/* Project includes */
#include "quadpid.hpp"

static const ctrl_quadpid_parameters_t ctrl_quadpid_params = {
    .linear_speed_pid = {
        .kp = 5.,
        .ki = 0.1,
        .kd = 0.,
        .ti = 0.,
        .previous_error = 0.
    },
    .angular_speed_pid = {
        .kp = 7,
        .ki = 0.1,
        .kd = 0,
        .ti = 0,
        .previous_error = 0.
    },
    .linear_pose_pid = {
        .kp = 0.5,
        .ki = 0,
        .kd = 0,
        .ti = 0.,
        .previous_error = 0.
    },
    .angular_pose_pid = {
        .kp = 4.,
        .ki = 0,
        .kd = 0,
        .ti = 0.,
        .previous_error = 0.
    },

    .min_distance_for_angular_switch = 2,       // mm,
    .min_angle_for_pose_reached = 1,            // deg,
    .min_angle_for_target_orientation = 2,            // deg,
    .regul = CTRL_REGUL_POSE_DIST,
};