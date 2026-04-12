// Copyright (C) 2026 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup     drivers_otos
/// @{
/// @file
/// @brief       SparkFun OTOS I2C driver class definition
/// @author      Gilles DOFFE <g.doffe@gmail.com>

#pragma once

#include "otos/OTOSRegisters.hpp"

#include "soft_i2c/soft_i2c.h"

namespace cogip {
namespace otos {

/// @brief Low-level driver for the SparkFun OTOS sensor (PAA5160E1 + IMU)
/// @details Communicates via software I2C (GPIO bitbanging)
class OTOS
{
  public:
    /// @brief Constructor
    /// @param i2c_dev Software I2C bus index (e.g., SOFT_I2C_DEV(0))
    /// @param i2c_addr 7-bit I2C address (default 0x17)
    explicit OTOS(soft_i2c_t i2c_dev, uint8_t i2c_addr = OTOS_DEFAULT_ADDR);

    /// @brief Initialize the sensor (verify product ID, soft reset, configure)
    /// @return 0 on success, negative on error
    int init();

    /// @brief Read pose, velocity and acceleration from sensor
    /// @return 0 on success, negative on I2C error
    int update();

    /// @brief Get latest pose (valid after update())
    const Pose2D& pose() const
    {
        return pose_;
    }

    /// @brief Get latest velocity (valid after update())
    const Pose2D& velocity() const
    {
        return velocity_;
    }

    /// @brief Get latest acceleration (valid after update())
    const Pose2D& acceleration() const
    {
        return acceleration_;
    }

    /// @brief Set the linear distance calibration scalar
    /// @param scalar Calibration factor (range 0.872 to 1.127)
    /// @return 0 on success, negative on error
    int set_linear_scalar(float scalar);

    /// @brief Set the angular heading calibration scalar
    /// @param scalar Calibration factor (range 0.872 to 1.127)
    /// @return 0 on success, negative on error
    int set_angular_scalar(float scalar);

    /// @brief Set the sensor mounting offset relative to robot center
    /// @param x_mm X offset in mm
    /// @param y_mm Y offset in mm
    /// @param h_deg Heading offset in degrees
    /// @return 0 on success, negative on error
    int set_offset(float x_mm, float y_mm, float h_deg);

    /// @brief Reset tracking to given position
    /// @param x_mm X position in mm
    /// @param y_mm Y position in mm
    /// @param h_deg Heading in degrees
    /// @return 0 on success, negative on error
    int set_position(float x_mm, float y_mm, float h_deg);

    /// @brief Run IMU calibration (blocking, ~612ms)
    /// @note The sensor must be stationary during calibration
    /// @param num_samples Number of calibration samples (default 255)
    /// @return 0 on success, negative on error
    int calibrate_imu(uint8_t num_samples = 255);

  private:
    soft_i2c_t i2c_dev_;
    uint8_t i2c_addr_;
    Pose2D pose_;
    Pose2D velocity_;
    Pose2D acceleration_;

    /// @brief Read a register block
    int read_regs(uint8_t reg, uint8_t* buf, size_t len);

    /// @brief Write a register block
    int write_regs(uint8_t reg, const uint8_t* buf, size_t len);

    /// @brief Read a single register byte
    int read_reg(uint8_t reg, uint8_t* value);

    /// @brief Write a single register byte
    int write_reg(uint8_t reg, uint8_t value);

    /// @brief Read 6 bytes (3x int16) and convert to Pose2D
    /// @param start_reg Starting register address
    /// @param out Output pose
    /// @param pos_scale Position scale factor (LSB to mm)
    /// @param heading_scale Heading scale factor (LSB to deg)
    /// @return 0 on success, negative on error
    int read_pose2d(uint8_t start_reg, Pose2D& out, float pos_scale, float heading_scale);

    /// @brief Write 6 bytes (3x int16) from Pose2D values
    /// @param start_reg Starting register address
    /// @param x_mm X in mm
    /// @param y_mm Y in mm
    /// @param h_deg Heading in degrees
    /// @param pos_scale mm to LSB scale
    /// @param heading_scale deg to LSB scale
    /// @return 0 on success, negative on error
    int write_pose2d(uint8_t start_reg, float x_mm, float y_mm, float h_deg, float pos_scale,
                     float heading_scale);
};

} // namespace otos
} // namespace cogip

/// @}
