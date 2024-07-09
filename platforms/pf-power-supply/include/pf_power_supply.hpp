/*
 * Copyright (C) 2024 COGIP Robotics association
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     platforms
 * @brief       Generic power supply definition
 *
 * @author      Mathis LECRIVAIN <lecrivain.mathis@gmail.com>
 */

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
// constexpr gpio_t pin_p3v3_pgood = GPIO_PIN(PORT_B, 1);
// constexpr gpio_t pin_cart_magnet_right = GPIO_PIN(PORT_C, 9);
#define P3V3_PGOOD_PIN GPIO_PIN(PORT_B, 1)
#define P5V0_PGOOD_PIN GPIO_PIN(PORT_C, 5)
#define P7V5_PGOOD_PIN GPIO_PIN(PORT_A, 7)
#define PxVx_PGOOD_PIN GPIO_PIN(PORT_A, 4)

#define EN_HIGH_POWER_PIN     GPIO_PIN(PORT_B, 3)
#define BATTERY_VALID_N_PIN   GPIO_PIN(PORT_B, 10)
#define DC_SUPPLY_VALID_N_PIN GPIO_PIN(PORT_B, 12)
/// @}

} // namespace power_supply
} // namespace pf
} // namespace cogip

/// @}
