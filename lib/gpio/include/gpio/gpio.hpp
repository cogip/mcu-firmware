///  Copyright (C) 2025 COGIP Robotics association
///
///  This file is subject to the terms and conditions of the GNU Lesser
///  General Public License v2.1. See the file LICENSE in the top level
///  directory for more details.
#pragma once

#include <periph/gpio.h>

///
/// @file
/// @defgroup    lib_gpio GPIO Library
/// @ingroup     lib
/// @brief       C++ class representing GPIO using RIOT GPIO driver.
///
/// @author      Mathis LECRIVAIN <lecrivain.mathis@gmail.com>
///
namespace cogip {

namespace gpio {

class GPIO
{
  public:
    /// @brief Initialize the given pin as general purpose input or output
    /// @param pin pin to initialize
    /// @param mode mode of the pin, see @ref gpio_mode_t
    GPIO(gpio_t pin, gpio_mode_t mode);

#if defined(MODULE_PERIPH_GPIO_IRQ)
    /// @brief Initialize a GPIO pin for external interrupt usage
    /// @param pin pin to initialize
    /// @param mode mode of the pin, see @ref gpio_mode_t
    /// @param flank define the active flank(s)
    /// @param cb callback that is called from interrupt context
    /// @param arg optional argument passed to the callback
    GPIO(gpio_t pin, gpio_mode_t mode, gpio_flank_t flank, gpio_cb_t cb, void* arg);
#endif

    /// @brief Get the current value of the given pin
    /// @return 0 when pin is LOW. >0 when pin is HIGH
    bool read(void) const;

    /// @brief Set the given pin to HIGH
    void set(void);

    /// @brief Set the given pin to LOW
    void clear(void);

    /// @brief Toggle the value of the given pin
    void toggle(void);

    /// @brief Set the given pin to the given value
    /// @param value value to set the pin to, 0 for LOW, HIGH otherwise
    void write(int value);

    /// @brief Test if pin is a valid pin and not declared as undefined.
    /// @return True when valid. False when invalid
    bool is_valid(void) const;

  private:
    gpio_t _pin; ///< [in] GPIO pin
};

} // namespace gpio

} // namespace cogip
