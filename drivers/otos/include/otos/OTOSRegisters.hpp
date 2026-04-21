// Copyright (C) 2026 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup     drivers_otos
/// @{
/// @file
/// @brief       SparkFun OTOS register map and data structures
/// @author      Gilles DOFFE <g.doffe@gmail.com>

#pragma once

#include <cstdint>

namespace cogip {
namespace otos {

/// I2C default address
constexpr uint8_t OTOS_DEFAULT_ADDR = 0x17;

/// Expected product ID
constexpr uint8_t OTOS_PRODUCT_ID = 0x5F;

/// @name Register addresses
/// @{
constexpr uint8_t REG_PRODUCT_ID = 0x00;
constexpr uint8_t REG_HW_VERSION = 0x01;
constexpr uint8_t REG_FW_VERSION = 0x02;
constexpr uint8_t REG_SCALAR_LINEAR = 0x04;
constexpr uint8_t REG_SCALAR_ANGULAR = 0x05;
constexpr uint8_t REG_IMU_CALIB = 0x06;
constexpr uint8_t REG_RESET = 0x07;
constexpr uint8_t REG_SIGNAL_PROCESS = 0x0E;
constexpr uint8_t REG_SELF_TEST = 0x0F;
constexpr uint8_t REG_OFFSET_XL = 0x10;
constexpr uint8_t REG_STATUS = 0x1F;
constexpr uint8_t REG_POS_XL = 0x20;
constexpr uint8_t REG_VEL_XL = 0x26;
constexpr uint8_t REG_ACC_XL = 0x2C;
constexpr uint8_t REG_POS_STDDEV_XL = 0x32;
constexpr uint8_t REG_VEL_STDDEV_XL = 0x38;
constexpr uint8_t REG_ACC_STDDEV_XL = 0x3E;
/// @}

/// @name Conversion factors
/// Position: int16, range +-10m, LSB = 10.0/32768 meters
/// Heading:  int16, range +-pi rad, LSB = pi/32768 radians
/// @{
constexpr float POS_LSB_TO_METERS = 10.0f / 32768.0f;
constexpr float HEADING_LSB_TO_RAD = 3.14159265358979323846f / 32768.0f;
constexpr float METERS_TO_MM = 1000.0f;
constexpr float RAD_TO_DEG = 180.0f / 3.14159265358979323846f;
/// @}

/// 2D pose data structure
struct Pose2D
{
    float x; ///< X position in mm
    float y; ///< Y position in mm
    float h; ///< Heading in degrees
};

} // namespace otos
} // namespace cogip

/// @}
