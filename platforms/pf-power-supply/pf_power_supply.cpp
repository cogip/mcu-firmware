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
#include "platform.hpp"
#include "pf_power_supply.hpp"

// Libraries includes
#include "gpio/gpio.hpp"

// Embedded Template Library includes
#include "etl/map.h"

// Protobuf messages
#include "PB_Emergency_stop.hpp"

namespace cogip
{
namespace pf
{
namespace power_supply
{

/// GPIOs handler thread stack
static char power_supply_thread_stack_[THREAD_STACKSIZE_DEFAULT];

/// Emergency stop state protobuf message
static PB_EmergencyStopState _pb_emergency_stop_state;

/// GPIO event queue
static event_queue_t event_queue_;

/// Emergency GPIO callback definition
static void emergency_stop_state_gpio_cb_(void *arg);
static void emergency_stop_state_event_cb_(event_t *event);

/// Emergency stop GPIO definition
static gpio::GPIO en_high_power_gpio_(pf::power_supply::EMERGENCY_STOP_PIN, GPIO_IN, GPIO_BOTH, emergency_stop_state_gpio_cb_, NULL);

static event_t events_handler_list[PowerSupplyEvent::POWER_SUPPLY_EVENT_MAX] = {
    [PowerSupplyEvent::POWER_SUPPLY_EVENT_EMERGENCY_STOP] = {
        .handler = emergency_stop_state_event_cb_,
    },
};

/// @brief Wait for a gpio flank and post the emergency stop event
/// @param arg 
static void emergency_stop_state_gpio_cb_([[maybe_unused]]void *arg)
{
    event_post(&event_queue_, &events_handler_list[PowerSupplyEvent::POWER_SUPPLY_EVENT_EMERGENCY_STOP]);
}

/// @brief Callback called inside the power supply thread context when emergency stop event is called.
///        This callback send the power supply gpio state over canpb.
/// @param event 
static void emergency_stop_state_event_cb_([[maybe_unused]]event_t *event)
{
    static canpb::CanProtobuf &canpb = pf_get_canpb();

    // Clear protobuf message
    _pb_emergency_stop_state.clear();

    // Clear protobuf message
    _pb_emergency_stop_state.mutable_pressed() = en_high_power_gpio_.read();
    
    // Send protobuf message
    if (!canpb.send_message(emergency_stop_state, &_pb_emergency_stop_state)) {
        std::cerr << "Error: emergency_stop_state message not sent" << std::endl;
    }
}

static void *power_supply_thread_([[maybe_unused]]void *args)
{
    (void)args;
    event_t *event;

    // Initialize event queue
    event_queue_init(&event_queue_);
    
    // Wait for event and call the dedicated handler
    while ((event = event_wait(&event_queue_))) {
        event->handler(nullptr);
    }

    return nullptr;
}

void pf_init_power_supply(void)
{
}

void pf_init_power_supply_tasks(void)
{
    // Power supply GPIO handling thread
    thread_create(power_supply_thread_stack_, sizeof(power_supply_thread_stack_), THREAD_PRIORITY_MAIN - 1,
        THREAD_CREATE_STACKTEST, power_supply_thread_, NULL, "GPIO handling thread");
}

} // namespace power_supply
} // namespace pf
} // namespace cogip

/// @}
