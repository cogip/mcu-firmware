// Copyright (C) 2024 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

#include "pf_actuators.hpp"
#include "pf_positional_actuators.hpp"

#include "AnalogServo.hpp"
#include "Motor.hpp"
#include "OnOff.hpp"
#include "PositionalActuator.hpp"

#include "actuators_motors_params.hpp"
#include "pca9685_params.hpp"
#include "platform.hpp"

#include "etl/map.h"
#include "etl/pool.h"

// RIOT includes
#include <event.h>
#include <ztimer.h>
#include <motor_driver.h>

namespace cogip {
namespace pf {
namespace actuators {
namespace positional_actuators {

// Actuator state protobuf message
static PB_ActuatorState _pb_actuator_state;

/// Positional actuator timeout thread stack
static char _positional_actuators_timeout_thread_stack[THREAD_STACKSIZE_DEFAULT];
/// Positional actuators timeout thread period (ms)
constexpr uint16_t _positional_actuators_timeout_thread_period_ms = 100;
/// GPIOs handler thread stack
static char _gpio_handling_thread_stack[THREAD_STACKSIZE_DEFAULT];

/// Motors memory pool
static etl::pool<Motor, COUNT> _motors_pool;
/// OnOff memory pool
static etl::pool<OnOff, COUNT> _onoff_pool;
/// Analog servomotor pool
static etl::pool<AnalogServo, COUNT> _analog_servo_pool;
/// Positional actuators map
static etl::map<cogip::pf::actuators::Enum, PositionalActuator *, 4*COUNT> _positional_actuators;

/// GPIOs event pool
static etl::pool<event_t, 20> _gpio_event_pool;
/// GPIOs pin map
static etl::map<event_t *, gpio_t, 20> _gpio_pins;
/// GPIOs event map
static etl::map<gpio_t, event_t *, 20> _gpio_events;

/// GPIO event queue
static event_queue_t _new_gpio_event_queue;

/// GPIOs interrupt callback
static void _gpio_cb(void *arg)
{
    (void)arg;
    //gpio_t pin = (gpio_t)arg;
    //event_post(&_new_gpio_event_queue, _gpio_events[pin]);
}

/// Init interruptable pin with pullup
static void init_interruptable_pin(gpio_t pin, gpio_mode_t mode, gpio_flank_t flank=GPIO_BOTH) {
    // Initialize the pin as input with pull-up, interrupt on falling edge
    if (gpio_init_int(pin, mode, flank, _gpio_cb, (void *)pin) != 0) {
        std::cerr << "Error: init pin " << pin << " failed" << std::endl;
        return;
    }

    _gpio_events[pin] = _gpio_event_pool.create();
    _gpio_events[pin]->list_node.next = nullptr;
    _gpio_pins[_gpio_events[pin]] = pin;
}

/// Init I2C PWM driver
static void _pca9685_init() {
    // Init PCA9685
    if (pca9685_init(&AnalogServo::pca9685_dev, &pca9685_params) != PCA9685_OK) {
        puts("Error: PCA9685 init failed!");
        return;
    }
    if (pca9685_pwm_init(&AnalogServo::pca9685_dev, pca9685_params.mode, pca9685_params.freq, pca9685_params.res) != pca9685_params.freq) {
        puts("Error: PCA9685 PWM init failed!");
        return;
    }
}

/// GPIOs handling thread
static void *_gpio_handling_thread(void *args)
{
    (void)args;
    event_t *event;

    // Initialize GPIOs event queue
    event_queue_init(&_new_gpio_event_queue);

    while ((event = event_wait(&_new_gpio_event_queue))) {
    }

    return nullptr;
}

void disable_all() {
    for (auto & iterator: _positional_actuators) {
        PositionalActuator *positional_actuator = iterator.second;
        positional_actuator->disable();
    }
}

static void *_positional_actuators_timeout_thread(void *args)
{
    (void)args;

    // Init loop iteration start time
    ztimer_now_t loop_start_time = ztimer_now(ZTIMER_MSEC);

    while (true) {
        for (auto & iterator: _positional_actuators) {
            PositionalActuator *positional_actuator = iterator.second;
            if (positional_actuator->timeout_period()) {
                if (!positional_actuator->decrement_timeout_period()) {
                    positional_actuator->disable();
                    positional_actuator->send_state();
                }
            }
        }

        // Wait thread period to end
        ztimer_periodic_wakeup(ZTIMER_MSEC, &loop_start_time, _positional_actuators_timeout_thread_period_ms);
    }

    return 0;
}

void reset_positional_actuators(void) {
    // Grips
    _positional_actuators[(cogip::pf::actuators::Enum)Enum::ANALOGSERVO_BOTTOM_GRIP_LEFT]->actuate(analogservo_grip_bottom_left_init_value);
    _positional_actuators[(cogip::pf::actuators::Enum)Enum::ANALOGSERVO_BOTTOM_GRIP_RIGHT]->actuate(analogservo_grip_bottom_right_init_value);
    _positional_actuators[(cogip::pf::actuators::Enum)Enum::ANALOGSERVO_TOP_GRIP_LEFT]->actuate(analogservo_grip_top_left_init_value);
    _positional_actuators[(cogip::pf::actuators::Enum)Enum::ANALOGSERVO_TOP_GRIP_RIGHT]->actuate(analogservo_grip_top_right_init_value);

    // Magnets
    _positional_actuators[(cogip::pf::actuators::Enum)Enum::CART_MAGNET_LEFT]->actuate(false);
    _positional_actuators[(cogip::pf::actuators::Enum)Enum::CART_MAGNET_RIGHT]->actuate(false);
}

void init() {
    // Init PWM I2C driver
    _pca9685_init();

    // AnalogServo init
    _positional_actuators[(cogip::pf::actuators::Enum)Enum::ANALOGSERVO_TOP_GRIP_LEFT] = _analog_servo_pool.create(
        (cogip::pf::actuators::Enum)Enum::ANALOGSERVO_TOP_GRIP_LEFT,
        0,
        send_state,
        PCA9586Channels::CHANNEL_ANALOGSERVO_TOP_GRIP_LEFT
    );

    _positional_actuators[(cogip::pf::actuators::Enum)Enum::ANALOGSERVO_TOP_GRIP_RIGHT] = _analog_servo_pool.create(
        (cogip::pf::actuators::Enum)Enum::ANALOGSERVO_TOP_GRIP_RIGHT,
        0,
        send_state,
        PCA9586Channels::CHANNEL_ANALOGSERVO_TOP_GRIP_RIGHT
    );
    _positional_actuators[(cogip::pf::actuators::Enum)Enum::ANALOGSERVO_BOTTOM_GRIP_LEFT] = _analog_servo_pool.create(
        (cogip::pf::actuators::Enum)Enum::ANALOGSERVO_BOTTOM_GRIP_LEFT,
        0,
        send_state,
        PCA9586Channels::CHANNEL_ANALOGSERVO_BOTTOM_GRIP_LEFT
    );
    _positional_actuators[(cogip::pf::actuators::Enum)Enum::ANALOGSERVO_BOTTOM_GRIP_RIGHT] = _analog_servo_pool.create(
        (cogip::pf::actuators::Enum)Enum::ANALOGSERVO_BOTTOM_GRIP_RIGHT,
        0,
        send_state,
        PCA9586Channels::CHANNEL_ANALOGSERVO_BOTTOM_GRIP_RIGHT
    );

    // OnOff init
    _positional_actuators[(cogip::pf::actuators::Enum)Enum::CART_MAGNET_LEFT] = _onoff_pool.create(
        (cogip::pf::actuators::Enum)Enum::CART_MAGNET_LEFT,
        0,
        send_state,
        false,
        true,
        pin_cart_magnet_left
    );
    _positional_actuators[(cogip::pf::actuators::Enum)Enum::CART_MAGNET_RIGHT] = _onoff_pool.create(
        (cogip::pf::actuators::Enum)Enum::CART_MAGNET_RIGHT,
        0,
        send_state,
        false,
        true,
        pin_cart_magnet_right
    );

    // Positional actuators timeout thread
    thread_create(
        _gpio_handling_thread_stack,
        sizeof(_gpio_handling_thread_stack),
        THREAD_PRIORITY_MAIN - 1,
        THREAD_CREATE_STACKTEST,
        _gpio_handling_thread,
        NULL,
        "GPIO handling thread"
    );

    // Positional actuators timeout thread
    thread_create(
        _positional_actuators_timeout_thread_stack,
        sizeof(_positional_actuators_timeout_thread_stack),
        THREAD_PRIORITY_MAIN - 1,
        THREAD_CREATE_STACKTEST,
        _positional_actuators_timeout_thread,
        NULL,
        "Positional actuators timeout thread"
    );
}

bool contains(cogip::pf::actuators::Enum id) {
    return _positional_actuators.contains(id);
}

PositionalActuator & get(cogip::pf::actuators::Enum id) {
    return *_positional_actuators[id];
}

void send_state(cogip::pf::actuators::Enum positional_actuator) {
    // Protobuf CAN interface
    static cogip::canpb::CanProtobuf & canpb = pf_get_canpb();

    // Send protobuf message
    _pb_actuator_state.clear();
    positional_actuators::get(positional_actuator).pb_copy(_pb_actuator_state.mutable_positional_actuator());
    if (!canpb.send_message(actuator_state_uuid, &_pb_actuator_state)) {
        std::cerr << "Error: actuator_state_uuid message not sent" << std::endl;
    }
}

void send_states() {
    for (auto const & [id, actuator] : _positional_actuators) {
        send_state(id);
    }
}

} // namespace positional_actuators
} // namespace actuators
} // namespace pf
} // namespace cogip
