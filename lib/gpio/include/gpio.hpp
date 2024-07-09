#pragma once

#include <cstdint>

/*
 * Copyright (C) 2024 COGIP Robotics association
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    lib_gpio power supply control platform
 * @ingroup     lib
 * @brief       C++ class representing GPIO using ROIT GPIO driver.
 *
 * @author      Mathis LECRIVAIN <lecrivain.mathis@gmail.com>
 */
namespace cogip
{

namespace gpio
{

#include <periph/gpio.h>

enum GPIOMode {
};

class GPIO
{
      public:
	/**
	 * @brief Initialize the given pin as general purpose input or output
	 *
	 * @param[in] pin       pin to initialize
	 * @param[in] mode      mode of the pin, see @c gpio_mode_t
	 */
	GPIO(gpio_t pin, gpio_mode_t mode) : _pin(pin), _mode(mode)
	{
		gpio_init(_pin, _mode);
	}

#if defined(MODULE_PERIPH_GPIO_IRQ)
	/**
	 * @brief Initialize a GPIO pin for external interrupt usage
	 *
	 * @param[in] pin       pin to initialize
	 * @param[in] mode      mode of the pin, see @c gpio_mode_t
	 * @param[in] flank     define the active flank(s)
	 * @param[in] cb        callback that is called from interrupt context
	 * @param[in] arg       optional argument passed to the callback
	 */
	GPIO(gpio_t pin, gpio_mode_t mode, gpio_flank_t flank, gpio_cb_t cb, void *arg)
		: _pin(pin), _mode(mode), _flank(flank), _cb(cb)
	{
		gpio_init_int(_pin, _mode, _flank, _cb, arg);
	}
#endif

	/**
	 * @brief   Get the current value of the given pin
	 *
	 * @return              0 when pin is LOW
	 * @return              >0 for HIGH
	 */
	int read(void) const
	{
		return gpio_read(_pin);
	}

	/**
	 * @brief   Set the given pin to HIGH
	 */
	void set(void)
	{
		gpio_set(_pin);
	}

	/**
	 * @brief   Set the given pin to LOW
	 */
	void clear(void)
	{
		gpio_clear(_pin);
	}

	/**
	 * @brief   Toggle the value of the given pin
	 */
	void toggle(void)
	{
		gpio_toggle(_pin);
	}

	/**
	 * @brief   Set the given pin to the given value
	 *
	 * @param[in] value     value to set the pin to, 0 for LOW, HIGH otherwise
	 */
	void write(int value)
	{
		gpio_write(_pin, value);
	}

	/**
	 * @brief   Test if pin is a valid pin and not declared as undefined.
	 */
	int is_valid(void)
	{
		return (_pin != GPIO_UNDEF);
	}

      private:
	bool _active_state; /*< [in] Consider On when GPIO is equal to active_state */
	gpio_t _pin;        /*< [in] GPIO pin */
	gpio_mode_t _mode;
#if defined(MODULE_PERIPH_GPIO_IRQ)
	gpio_flank_t _flank;
	gpio_cb_t _cb;
#endif
};

} // namespace gpio

} // namespace cogip