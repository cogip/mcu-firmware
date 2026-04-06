// Copyright (C) 2026 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @file
/// @brief OTOS sensor test application
/// @details Initializes the OTOS sensor, calibrates the IMU, then
///          continuously prints pose data for validation.

#include "log.h"
#include "otos/OTOS.hpp"
#include "ztimer.h"

static cogip::otos::OTOS otos(I2C_DEV(0), 0x17);

int main(void)
{
    LOG_INFO("OTOS test application\n");

    int ret = otos.init();
    if (ret < 0) {
        LOG_ERROR("OTOS init failed: %d\n", ret);
        return 1;
    }

    LOG_INFO("Calibrating IMU (keep sensor stationary)...\n");
    ret = otos.calibrate_imu();
    if (ret < 0) {
        LOG_ERROR("IMU calibration failed: %d\n", ret);
        return 1;
    }

    LOG_INFO("Starting pose readout (move sensor to see values change)\n");

    while (1) {
        ret = otos.update();
        if (ret < 0) {
            LOG_ERROR("OTOS update failed: %d\n", ret);
        } else {
            const auto& pose = otos.pose();
            LOG_INFO("x=%.1f mm  y=%.1f mm  h=%.1f deg\n", static_cast<double>(pose.x),
                     static_cast<double>(pose.y), static_cast<double>(pose.h));
        }

        ztimer_sleep(ZTIMER_MSEC, 100);
    }

    return 0;
}
