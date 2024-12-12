///  Copyright (C) 2024 COGIP Robotics association
///
///  This file is subject to the terms and conditions of the GNU Lesser
///  General Public License v2.1. See the file LICENSE in the top level
///  directory for more details.

///  @ingroup     platforms
///  @brief       power supply platform
///
///  @author      Mathis LECRIVAIN <lecrivain.mathis@gmail.com>

// Standard includes
#include <cstdint>

// RIOT includes
#include <event.h>
#include <ztimer.h>

// Platform includes
#include "pf_power_supply.hpp"

// Libraries includes
#include "gpio/gpio.hpp"

// Embedded Template Library includes
#include "etl/map.h"

// Protobuf messages
#include "PB_Power_supply.hpp"

namespace cogip
{
namespace pf
{
namespace power_supply
{
    
/// GPIOs handler thread stack
static char _gpio_handling_thread_stack[THREAD_STACKSIZE_DEFAULT];

static void pxVx_error_cb_(void *arg);

// PGood GPIO objects creation
static gpio::GPIO p3V3_pgood_gpio_(pf::power_supply::P3V3_PGOOD_PIN, GPIO_IN, GPIO_FALLING, pxVx_error_cb_, NULL);
static gpio::GPIO p5V0_pgood_gpio_(pf::power_supply::P5V0_PGOOD_PIN, GPIO_IN, GPIO_FALLING, pxVx_error_cb_, NULL);
static gpio::GPIO p7V5_pgood_gpio_(pf::power_supply::P7V5_PGOOD_PIN, GPIO_IN, GPIO_FALLING, pxVx_error_cb_, NULL);
static gpio::GPIO pxVx_pgood_gpio_(pf::power_supply::PxVx_PGOOD_PIN, GPIO_IN, GPIO_FALLING, pxVx_error_cb_, NULL);

// Map protobuf PGood states enum to it's GPIO objects
static etl::map<PB_PGoodTypeEnum, const gpio::GPIO*, pf::power_supply::PGOOD_PIN_NUMBER> pgood_gpios_{
    etl::pair{PB_PGoodTypeEnum::P3V3_PGOOD, &p3V3_pgood_gpio_},
    etl::pair{PB_PGoodTypeEnum::P5V0_PGOOD, &p5V0_pgood_gpio_},
    etl::pair{PB_PGoodTypeEnum::P7V5_PGOOD, &p7V5_pgood_gpio_},
    etl::pair{PB_PGoodTypeEnum::PxVx_PGOOD, &pxVx_pgood_gpio_}
};

static void high_power_state_cb_(void *arg);

static gpio::GPIO en_high_power_gpio_(pf::power_supply::EN_HIGH_POWER_PIN, GPIO_IN, GPIO_BOTH, high_power_state_cb_, NULL);

static gpio::GPIO battery_valid_n_gpio_(pf::power_supply::BATTERY_VALID_N_PIN, GPIO_IN);
static gpio::GPIO dc_supply_valid_n_gpio_(pf::power_supply::DC_SUPPLY_VALID_N_PIN, GPIO_IN);

static void *_gpio_handling_thread(void *args);

/// @brief 
/// @param arg 
static void pxVx_error_cb_([[maybe_unused]]void *arg)
{
    
}

/// @brief 
/// @param arg 
static void high_power_state_cb_([[maybe_unused]]void *arg)
{
    
}

static void *_gpio_handling_thread([[maybe_unused]]void *args)
{
    //(void)args;
    //event_t *event;

    //// Initialize GPIOs event queue
    //event_queue_init(&_new_gpio_event_queue);

    //while ((event = event_wait(&_new_gpio_event_queue))) {
    //    gpio_t pin = _gpio_pins[event];

    //    BoolSensor* bool_sensor = _bool_sensors_events[event];
    //    if (bool_sensor) {
    //        bool_sensor->set_state(!gpio_read(pin));
    //        bool_sensor->send_state();
    //    }
    //}

    return nullptr;
}

void pf_init_power_supply(void)
{
    // TODO
}

void pf_init_power_supply_tasks(void)
{
    // Power supply GPIO handling thread
    thread_create(_gpio_handling_thread_stack, sizeof(_gpio_handling_thread_stack), THREAD_PRIORITY_MAIN - 1,
        THREAD_CREATE_STACKTEST, _gpio_handling_thread, NULL, "GPIO handling thread");
}

} // namespace power_supply
} // namespace pf
} // namespace cogip

/// @}
