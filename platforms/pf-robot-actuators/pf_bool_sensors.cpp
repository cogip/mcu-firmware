// Copyright (C) 2024 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

#include "pf_sensors.hpp"
#include "pf_bool_sensors.hpp"

#include "BoolSensor.hpp"

#include "platform.hpp"

#include "etl/map.h"
#include "etl/pool.h"

// RIOT includes
#include <event.h>
#include <ztimer.h>
#include <motor_driver.h>

namespace cogip {
namespace pf {
namespace sensors {
namespace bool_sensors {

// sensor state protobuf message
static PB_ActuatorState _pb_sensor_state;

/// GPIOs handler thread stack
static char _gpio_handling_thread_stack[THREAD_STACKSIZE_DEFAULT];

/// BoolSensor memory pool
static etl::pool<BoolSensor, COUNT> _bool_sensors_pool;
/// Bool sensors map
static etl::map<cogip::pf::sensors::Enum, BoolSensor *, COUNT> _bool_sensors;

/// GPIOs event pool
static etl::pool<event_t, COUNT> _gpio_event_pool;
/// GPIOs pin map
static etl::map<event_t *, gpio_t, COUNT> _gpio_pins;
/// GPIOs event map
static etl::map<gpio_t, event_t *, COUNT> _gpio_events;
/// BoolSensor map
static etl::map<event_t *, BoolSensor *, COUNT> _bool_sensors_events;

/// GPIO event queue
static event_queue_t _new_gpio_event_queue;

/// GPIOs interrupt callback
static void _gpio_cb(void *arg)
{
    (void)arg;
    gpio_t pin = (gpio_t)arg;
    event_post(&_new_gpio_event_queue, _gpio_events[pin]);
}

/// Init interruptable pin with pullup
static void init_interruptable_pin(gpio_t pin, gpio_mode_t mode, gpio_flank_t flank=GPIO_BOTH, BoolSensor* bool_sensor = nullptr) {
    // Initialize the pin as input with pull-up, interrupt on falling edge
    if (gpio_init_int(pin, mode, flank, _gpio_cb, (void *)pin) != 0) {
        std::cerr << "Error: init pin " << pin << " failed" << std::endl;
        return;
    }

    _gpio_events[pin] = _gpio_event_pool.create();
    _gpio_events[pin]->list_node.next = nullptr;
    _bool_sensors_events[_gpio_events[pin]] = bool_sensor;
    _gpio_pins[_gpio_events[pin]] = pin;
}

/// GPIOs handling thread
static void *_gpio_handling_thread(void *args)
{
    (void)args;
    event_t *event;

    // Initialize GPIOs event queue
    event_queue_init(&_new_gpio_event_queue);

    while ((event = event_wait(&_new_gpio_event_queue))) {
        gpio_t pin = _gpio_pins[event];

        BoolSensor* bool_sensor = _bool_sensors_events[event];
        if (bool_sensor) {
            bool_sensor->set_state(!gpio_read(pin));
            bool_sensor->send_state();
        }
    }

    return nullptr;
}

/// Initialize BoolSensors
void init() {
    _bool_sensors[(cogip::pf::sensors::Enum)Enum::BOTTOM_GRIP_LEFT] = _bool_sensors_pool.create((cogip::pf::sensors::Enum)Enum::BOTTOM_GRIP_LEFT, false, send_state);
    _bool_sensors[(cogip::pf::sensors::Enum)Enum::BOTTOM_GRIP_RIGHT] = _bool_sensors_pool.create((cogip::pf::sensors::Enum)Enum::BOTTOM_GRIP_RIGHT, false, send_state);
    _bool_sensors[(cogip::pf::sensors::Enum)Enum::TOP_GRIP_LEFT] = _bool_sensors_pool.create((cogip::pf::sensors::Enum)Enum::TOP_GRIP_LEFT, false, send_state);
    _bool_sensors[(cogip::pf::sensors::Enum)Enum::TOP_GRIP_RIGHT] = _bool_sensors_pool.create((cogip::pf::sensors::Enum)Enum::TOP_GRIP_RIGHT, false, send_state);
    _bool_sensors[(cogip::pf::sensors::Enum)Enum::MAGNET_LEFT] = _bool_sensors_pool.create((cogip::pf::sensors::Enum)Enum::MAGNET_LEFT, false, send_state);
    _bool_sensors[(cogip::pf::sensors::Enum)Enum::MAGNET_RIGHT] = _bool_sensors_pool.create((cogip::pf::sensors::Enum)Enum::MAGNET_RIGHT, false, send_state);

    init_interruptable_pin(GPIO_PIN(PORT_B, 9), GPIO_IN_PU, GPIO_BOTH, _bool_sensors[(cogip::pf::sensors::Enum)Enum::BOTTOM_GRIP_LEFT]);
    init_interruptable_pin(GPIO_PIN(PORT_C, 14), GPIO_IN_PU, GPIO_BOTH, _bool_sensors[(cogip::pf::sensors::Enum)Enum::BOTTOM_GRIP_RIGHT]);
    init_interruptable_pin(GPIO_PIN(PORT_C, 0), GPIO_IN_PU, GPIO_BOTH, _bool_sensors[(cogip::pf::sensors::Enum)Enum::TOP_GRIP_LEFT]);
    init_interruptable_pin(GPIO_PIN(PORT_C, 13), GPIO_IN_PU, GPIO_BOTH, _bool_sensors[(cogip::pf::sensors::Enum)Enum::TOP_GRIP_RIGHT]);
    init_interruptable_pin(GPIO_PIN(PORT_C, 15), GPIO_IN_PU, GPIO_BOTH, _bool_sensors[(cogip::pf::sensors::Enum)Enum::MAGNET_LEFT]);
    init_interruptable_pin(GPIO_PIN(PORT_B, 7), GPIO_IN_PU, GPIO_BOTH, _bool_sensors[(cogip::pf::sensors::Enum)Enum::MAGNET_RIGHT]);

    // BoolSensor GPIO handling thread
    thread_create(
        _gpio_handling_thread_stack,
        sizeof(_gpio_handling_thread_stack),
        THREAD_PRIORITY_MAIN - 1,
        THREAD_CREATE_STACKTEST,
        _gpio_handling_thread,
        NULL,
        "GPIO handling thread"
    );
}

/// Get a BoolSensor by its ID
BoolSensor & get(cogip::pf::sensors::Enum id) {
    return *_bool_sensors[id];
}

bool contains(cogip::pf::sensors::Enum id) {
    return _bool_sensors.contains(id);
}

void send_state(cogip::pf::sensors::Enum id) {
    // Protobuf CAN interface
    static cogip::canpb::CanProtobuf & canpb = pf_get_canpb();

    std::cout << "send state for ID " << static_cast<int>(id) << std::endl;

    // Send protobuf message
    _pb_sensor_state.clear();
    bool_sensors::get(id).pb_copy(_pb_sensor_state.mutable_bool_sensor());
    if (!canpb.send_message(actuator_state_uuid, &_pb_sensor_state)) {
        std::cerr << "Error: sensor_state_uuid message not sent" << std::endl;
    }
}

void send_states() {
    for (auto const & [id, sensor] : _bool_sensors) {
        send_state(id);
    }
}

} // namespace bool_sensors
} // namespace sensors
} // namespace pf
} // namespace cogip
