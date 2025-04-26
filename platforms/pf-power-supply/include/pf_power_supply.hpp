///  Copyright (C) 2024 COGIP Robotics association
///
///  This file is subject to the terms and conditions of the GNU Lesser
///  General Public License v2.1. See the file LICENSE in the top level
///  directory for more details.

///  @ingroup     platforms
///  @brief       Generic power supply definition
///
///  @author      Mathis LECRIVAIN <lecrivain.mathis@gmail.com>

#pragma once

// Standard includes
#include <cstdint>
#include <periph/gpio.h>

namespace cogip
{
namespace pf
{
namespace power_supply
{

/// Power supply input pins
/// @{

// Power supply state pins
constexpr gpio_t P3V3_PGOOD_PIN = GPIO_PIN(PORT_B, 1);
constexpr gpio_t P5V0_PGOOD_PIN = GPIO_PIN(PORT_C, 5);
constexpr gpio_t P7V5_PGOOD_PIN = GPIO_PIN(PORT_A, 7);
constexpr gpio_t PxVx_PGOOD_PIN = GPIO_PIN(PORT_A, 4);

// Emergency stop pin
constexpr gpio_t EMERGENCY_STOP_PIN     = GPIO_PIN(PORT_B, 3);

// Power supply source pins
constexpr gpio_t BATTERY_VALID_N_PIN   = GPIO_PIN(PORT_B, 10);
constexpr gpio_t DC_SUPPLY_VALID_N_PIN = GPIO_PIN(PORT_B, 12);

/// @}

enum PowerSupplyEvent {
    POWER_SUPPLY_EVENT_EMERGENCY_STOP = 0,
    // Add more event here
    POWER_SUPPLY_EVENT_MAX = 1,
};

/// @brief Initialize power supply platform layer
void pf_init_power_supply(void);

/// @brief  Initialize power supply internal tasks
void pf_init_power_supply_tasks(void);

} // namespace power_supply
} // namespace pf
} // namespace cogip

/// @}
