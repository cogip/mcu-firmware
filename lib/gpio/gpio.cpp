/// Copyright (C) 2025 COGIP Robotics association
///
/// This file is subject to the terms and conditions of the GNU Lesser
/// General Public License v2.1. See the file LICENSE in the top level
/// directory for more details.

#include "gpio/gpio.hpp"

namespace cogip {

namespace gpio {

GPIO::GPIO(gpio_t pin, gpio_mode_t mode) : _pin(pin)
{
    gpio_init(_pin, mode);
}

#if defined(MODULE_PERIPH_GPIO_IRQ)
GPIO::GPIO(gpio_t pin, gpio_mode_t mode, gpio_flank_t flank, gpio_cb_t cb, void* arg) : _pin(pin)
{
    gpio_init_int(_pin, mode, flank, cb, arg);
}
#endif

bool GPIO::read(void) const
{
    return gpio_read(_pin);
}

void GPIO::set(void)
{
    gpio_set(_pin);
}

void GPIO::clear(void)
{
    gpio_clear(_pin);
}

void GPIO::toggle(void)
{
    gpio_toggle(_pin);
}

void GPIO::write(int value)
{
    gpio_write(_pin, value);
}

bool GPIO::is_valid(void) const
{
    return gpio_is_valid(_pin);
}

} // namespace gpio

} // namespace cogip
