// Copyright (C) 2026 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup     drivers_otos
/// @{
/// @file
/// @brief       SparkFun OTOS I2C driver implementation
/// @author      Gilles DOFFE <g.doffe@gmail.com>

#include "otos/OTOS.hpp"

#include "log.h"
#include "ztimer.h"

#include <cstring>

#define ENABLE_DEBUG 0
#include <debug.h>

namespace cogip {
namespace otos {

/// Position LSB to mm conversion factor
static constexpr float POS_LSB_TO_MM = POS_LSB_TO_METERS * METERS_TO_MM;

/// Heading LSB to degrees conversion factor
static constexpr float HEADING_LSB_TO_DEG = HEADING_LSB_TO_RAD * RAD_TO_DEG;

/// Inverse factors for writing
static constexpr float MM_TO_POS_LSB = 1.0f / POS_LSB_TO_MM;
static constexpr float DEG_TO_HEADING_LSB = 1.0f / HEADING_LSB_TO_DEG;

OTOS::OTOS(soft_i2c_t i2c_dev, uint8_t i2c_addr)
    : i2c_dev_(i2c_dev), i2c_addr_(i2c_addr), pose_{}, velocity_{}, acceleration_{}
{
}

int OTOS::init()
{
    // Initialize software I2C bus
    soft_i2c_init(i2c_dev_);

    // Verify product ID
    uint8_t product_id = 0;
    int ret = read_reg(REG_PRODUCT_ID, &product_id);
    if (ret < 0) {
        LOG_ERROR("OTOS: failed to read product ID (I2C error %d)\n", ret);
        return ret;
    }
    if (product_id != OTOS_PRODUCT_ID) {
        LOG_ERROR("OTOS: unexpected product ID 0x%02X (expected 0x%02X)\n", product_id,
                  OTOS_PRODUCT_ID);
        return -1;
    }

    LOG_INFO("OTOS: product ID verified (0x%02X)\n", product_id);

    // Soft reset
    ret = write_reg(REG_RESET, 0x01);
    if (ret < 0) {
        LOG_ERROR("OTOS: soft reset failed\n");
        return ret;
    }
    ztimer_sleep(ZTIMER_MSEC, 5);

    LOG_INFO("OTOS: initialized\n");
    return 0;
}

int OTOS::update()
{
    int ret = read_pose2d(REG_POS_XL, pose_, POS_LSB_TO_MM, HEADING_LSB_TO_DEG);
    if (ret < 0) {
        return ret;
    }

    ret = read_pose2d(REG_VEL_XL, velocity_, POS_LSB_TO_MM, HEADING_LSB_TO_DEG);
    if (ret < 0) {
        return ret;
    }

    ret = read_pose2d(REG_ACC_XL, acceleration_, POS_LSB_TO_MM, HEADING_LSB_TO_DEG);
    if (ret < 0) {
        return ret;
    }

    DEBUG("OTOS: x=%.1f y=%.1f h=%.1f\n", static_cast<double>(pose_.x),
          static_cast<double>(pose_.y), static_cast<double>(pose_.h));

    return 0;
}

int OTOS::set_linear_scalar(float scalar)
{
    // Scalar stored as int8: (scalar - 1.0) * 1000, range 0.872-1.127
    auto raw = static_cast<int8_t>((scalar - 1.0f) * 1000.0f + 0.5f);
    return write_reg(REG_SCALAR_LINEAR, static_cast<uint8_t>(raw));
}

int OTOS::set_angular_scalar(float scalar)
{
    auto raw = static_cast<int8_t>((scalar - 1.0f) * 1000.0f + 0.5f);
    return write_reg(REG_SCALAR_ANGULAR, static_cast<uint8_t>(raw));
}

int OTOS::set_offset(float x_mm, float y_mm, float h_deg)
{
    return write_pose2d(REG_OFFSET_XL, x_mm, y_mm, h_deg, MM_TO_POS_LSB, DEG_TO_HEADING_LSB);
}

int OTOS::set_position(float x_mm, float y_mm, float h_deg)
{
    return write_pose2d(REG_POS_XL, x_mm, y_mm, h_deg, MM_TO_POS_LSB, DEG_TO_HEADING_LSB);
}

int OTOS::calibrate_imu(uint8_t num_samples)
{
    LOG_INFO("OTOS: starting IMU calibration (%u samples)...\n", num_samples);

    int ret = write_reg(REG_IMU_CALIB, num_samples);
    if (ret < 0) {
        LOG_ERROR("OTOS: failed to start IMU calibration\n");
        return ret;
    }

    // Poll until calibration is complete (register reads 0 when done)
    // Each sample takes ~2.4ms, timeout after 2x expected duration
    uint32_t timeout_ms = static_cast<uint32_t>(num_samples) * 5;
    uint32_t elapsed = 0;
    while (elapsed < timeout_ms) {
        ztimer_sleep(ZTIMER_MSEC, 10);
        elapsed += 10;

        uint8_t status = 0;
        ret = read_reg(REG_IMU_CALIB, &status);
        if (ret < 0) {
            LOG_ERROR("OTOS: failed to read calibration status\n");
            return ret;
        }
        if (status == 0) {
            LOG_INFO("OTOS: IMU calibration complete (%lu ms)\n",
                     static_cast<unsigned long>(elapsed));
            return 0;
        }
    }

    LOG_ERROR("OTOS: IMU calibration timeout\n");
    return -1;
}

int OTOS::read_regs(uint8_t reg, uint8_t* buf, size_t len)
{
    soft_i2c_acquire(i2c_dev_);
    int ret = soft_i2c_read_regs(i2c_dev_, i2c_addr_, reg, buf, len);
    soft_i2c_release(i2c_dev_);
    return ret;
}

int OTOS::write_regs(uint8_t reg, const uint8_t* buf, size_t len)
{
    soft_i2c_acquire(i2c_dev_);
    int ret = soft_i2c_write_regs(i2c_dev_, i2c_addr_, reg, buf, len);
    soft_i2c_release(i2c_dev_);
    return ret;
}

int OTOS::read_reg(uint8_t reg, uint8_t* value)
{
    return read_regs(reg, value, 1);
}

int OTOS::write_reg(uint8_t reg, uint8_t value)
{
    return write_regs(reg, &value, 1);
}

int OTOS::read_pose2d(uint8_t start_reg, Pose2D& out, float pos_scale, float heading_scale)
{
    uint8_t buf[6];
    int ret = read_regs(start_reg, buf, sizeof(buf));
    if (ret < 0) {
        return ret;
    }

    auto raw_x = static_cast<int16_t>(buf[0] | (buf[1] << 8));
    auto raw_y = static_cast<int16_t>(buf[2] | (buf[3] << 8));
    auto raw_h = static_cast<int16_t>(buf[4] | (buf[5] << 8));

    out.x = static_cast<float>(raw_x) * pos_scale;
    out.y = static_cast<float>(raw_y) * pos_scale;
    out.h = static_cast<float>(raw_h) * heading_scale;

    return 0;
}

int OTOS::write_pose2d(uint8_t start_reg, float x_mm, float y_mm, float h_deg, float pos_scale,
                       float heading_scale)
{
    auto raw_x = static_cast<int16_t>(x_mm * pos_scale);
    auto raw_y = static_cast<int16_t>(y_mm * pos_scale);
    auto raw_h = static_cast<int16_t>(h_deg * heading_scale);

    uint8_t buf[6] = {
        static_cast<uint8_t>(raw_x & 0xFF), static_cast<uint8_t>((raw_x >> 8) & 0xFF),
        static_cast<uint8_t>(raw_y & 0xFF), static_cast<uint8_t>((raw_y >> 8) & 0xFF),
        static_cast<uint8_t>(raw_h & 0xFF), static_cast<uint8_t>((raw_h >> 8) & 0xFF)};

    return write_regs(start_reg, buf, sizeof(buf));
}

} // namespace otos
} // namespace cogip

/// @}
