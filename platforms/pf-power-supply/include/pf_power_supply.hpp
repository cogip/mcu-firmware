///  Copyright (C) 2025 COGIP Robotics association
///
///  This file is subject to the terms and conditions of the GNU Lesser
///  General Public License v2.1. See the file LICENSE in the top level
///  directory for more details.

///
///  @file
///  @defgroup    pf_power_supply Power Supply Platform
///  @ingroup     platforms
///  @brief       Generic power supply definition
///
///  @author      Mathis LECRIVAIN <lecrivain.mathis@gmail.com>
///  @{

#pragma once

// Standard includes
#include <cstdint>
#include <periph/gpio.h>

#include "board.h"

namespace cogip {
namespace pf {
namespace power_supply {

/// Power supply input pins
/// @{

// PGood pins
constexpr gpio_t P3V3_PGOOD_PIN = GPIO_PIN(PORT_B, 1);
constexpr gpio_t P5V0_PGOOD_PIN = GPIO_PIN(PORT_C, 5);
constexpr gpio_t P7V5_PGOOD_PIN = GPIO_PIN(PORT_A, 7);
constexpr gpio_t PxVx_PGOOD_PIN = GPIO_PIN(PORT_A, 4);

// Emergency stop pin
constexpr gpio_t EN_HIGH_POWER_PIN = GPIO_PIN(PORT_B, 3);

// Power source pins
constexpr gpio_t BATTERY_VALID_N_PIN = GPIO_PIN(PORT_B, 10);
constexpr gpio_t DC_SUPPLY_VALID_N_PIN = GPIO_PIN(PORT_B, 12);

/// @}

/// @brief Initialize power supply GPIO platform
void pf_init_power_supply(void);

/// @brief Create and start power supply GPIO monitoring task
/// @details Creates a dedicated thread that monitors all power supply GPIO states
///          and displays both initial states and real-time changes
void pf_init_power_supply_tasks(void);

/// @brief Send power rails status over CAN
void send_power_rails_status(void);

/// @brief Send power source status over CAN
void send_power_source_status(void);

/// @brief Send emergency stop status over CAN
void send_emergency_stop_status(void);

} // namespace power_supply
} // namespace pf
} // namespace cogip

/// @}
