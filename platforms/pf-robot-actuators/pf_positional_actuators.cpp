// Copyright (C) 2023 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

#include "pf_actuators.hpp"
#include "pf_positional_actuators.hpp"

#include "AnalogServo.hpp"
#include "LxMotor.hpp"
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

// Motion control motor driver
//static motor_driver_t actuators_motors_driver;

// Positional actuator protobuf message
static PB_PositionalActuator _pb_positional_actuator;

/// Positional actuator timeout thread stack
static char _positional_actuators_timeout_thread_stack[THREAD_STACKSIZE_DEFAULT];
/// Positional actuators timeout thread period (ms)
constexpr uint16_t _positional_actuators_timeout_thread_period_ms = 100;
/// GPIOs handler thread stack
//static char _gpio_handling_thread_stack[THREAD_STACKSIZE_DEFAULT];

/// Motors memory pool
static etl::pool<Motor, COUNT> _motors_pool;
/// OnOff memory pool
static etl::pool<OnOff, COUNT> _onoff_pool;
/// Analog servomotor pool
static etl::pool<AnalogServo, COUNT> _analog_servo_pool;
/// Numerical servomotor pool
static etl::pool<LxMotor, COUNT> _lxmotor_pool;
/// Positional actuators map
static etl::map<Enum, PositionalActuator *, 4*COUNT> _positional_actuators;

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
    gpio_t pin = (int)arg;
    event_post(&_new_gpio_event_queue, _gpio_events[pin]);
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

/// Check central lift bottom limit switch
static int check_limit_switch_bottom_lift() {
    return gpio_read(pin_limit_switch_bottom_lift);
}

/// Check central lift top limit switch
static int check_limit_switch_top_lift() {
    return gpio_read(pin_limit_switch_top_lift);
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
        gpio_t pin = _gpio_pins[event];

        switch (pin)
        {
        case pin_limit_switch_bottom_lift:
            std::cout << "pin_limit_switch_bottom_lift triggered" << std::endl;
            _positional_actuators[Enum::MOTOR_BOTTOM_LIFT]->disable_on_check();
            break;
        case pin_limit_switch_top_lift:
            std::cout << "pin_limit_switch_top_lift triggered" << std::endl;
            _positional_actuators[Enum::MOTOR_TOP_LIFT]->disable_on_check();
            break;
        default:
            std::cout << "INT: external interrupt from pin " << pin << std::endl;
            break;
        }
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
                }
            }
        }

        // Wait thread period to end
        ztimer_periodic_wakeup(ZTIMER_MSEC, &loop_start_time, _positional_actuators_timeout_thread_period_ms);
    }

    return 0;
}

void init(uart_half_duplex_t *lx_stream) {
    (void)lx_stream;
    // Half duplex stream that must have been initialized previously
    LxMotor::lx_stream = lx_stream;

    // Init PWM I2C driver
    _pca9685_init();

    // Motors driver init
    //motor_driver_init(&actuators_motors_driver, &actuators_motors_params);
    //_positional_actuators[Enum::MOTOR_BOTTOM_LIFT] = _motors_pool.create(
    //    Enum::MOTOR_BOTTOM_LIFT,
    //    GroupEnum::NO_GROUP,
    //    0,
    //    default_timeout_period_motor_bottom_lift,
    //    &actuators_motors_driver,
    //    actuator_central_lift_motor,
    //    nullptr,
    //    check_limit_switch_bottom_lift);

    // AnalogServo init
    _positional_actuators[Enum::ANALOGSERVO_BOTTOM_ARM_LEFT] = _analog_servo_pool.create(
        Enum::ANALOGSERVO_BOTTOM_ARM_LEFT,
        GroupEnum::NO_GROUP,
        0,
        0,
        PCA9586Channels::CHANNEL_ANALOGSERVO_BOTTOM_ARM_LEFT
    );
    static_cast<AnalogServo*>(_positional_actuators[Enum::ANALOGSERVO_BOTTOM_ARM_LEFT])->add_position(analog_servomotor_bottom_arm_left_closed);
    static_cast<AnalogServo*>(_positional_actuators[Enum::ANALOGSERVO_BOTTOM_ARM_LEFT])->add_position(analog_servomotor_bottom_arm_left_opened);

    // Positional actuators timeout thread
    //thread_create(
    //    _gpio_handling_thread_stack,
    //    sizeof(_gpio_handling_thread_stack),
    //    THREAD_PRIORITY_MAIN - 1,
    //    THREAD_CREATE_STACKTEST,
    //    _gpio_handling_thread,
    //    NULL,
    //    "GPIO handling thread"
    //);

    // Positional actuators timeout thread
    thread_create(
        _positional_actuators_timeout_thread_stack,
        sizeof(_positional_actuators_timeout_thread_stack),
        THREAD_PRIORITY_MAIN - 1,
        THREAD_CREATE_STACKTEST,
        _positional_actuators_timeout_thread,
        NULL,
        "Positional acturators timeout thread"
    );
}

PositionalActuator & get(Enum id) {
    return *_positional_actuators[id];
}

void send_state(Enum positional_actuator) {
    // Protobuf CAN interface
    static cogip::canpb::CanProtobuf & canpb = pf_get_canpb();

    // Send protobuf message
    _pb_positional_actuator.clear();
    positional_actuators::get(positional_actuator).pb_copy(_pb_positional_actuator);
    if (!canpb.send_message(actuator_state_uuid, &_pb_positional_actuator)) {
        std::cerr << "Error: actuator_state_uuid message not sent" << std::endl;
    }
}

void pb_copy(PB_Message & pb_message) {
    // cppcheck-suppress unusedVariable
    for (auto const & [id, actuator] : _positional_actuators) {
        actuator->pb_copy(pb_message.get(pb_message.get_length()));
    }
}

} // namespace actuators
} // namespace actuators
} // namespace pf
} // namespace cogip