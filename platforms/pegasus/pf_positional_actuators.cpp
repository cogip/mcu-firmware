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

#include "pca9685_params.h"
#include "pcf857x_params.h"
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

/// PCF857X I2C GPIOs expander
static pcf857x_t pcf857x_dev;

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
/// Analog servomotor pool
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

/// GPIO expander writer wrapper
void pf_pcf857x_gpio_write(gpio_t pin, int value) {
    pcf857x_gpio_write(&pcf857x_dev, pin, value);
}

/// GPIO expander writer wrapper
int pf_pcf857x_gpio_read(gpio_t pin) {
    return pcf857x_gpio_read(&pcf857x_dev, pin);
}

/// Init GPIO expander interruptable pin
static void init_interruptable_pin(gpio_t pin, bool pullup=true, gpio_flank_t flank=GPIO_BOTH) {
    // Initialize the pin as input with pull-up, interrupt on falling edge
    if (gpio_init_int(pin, pullup ? GPIO_IN_PU : GPIO_IN, flank, _gpio_cb, (void *)pin) != 0) {
        std::cerr << "Error: init pin " << pin << " failed" << std::endl;
        return;
    }

    _gpio_events[pin] = _gpio_event_pool.create();
    _gpio_events[pin]->list_node.next = nullptr;
    _gpio_pins[_gpio_events[pin]] = pin;
}

/// Init GPIO expander interruptable pin
static void pcf857x_init_interruptable_pin(gpio_t pin, bool pullup=true, gpio_flank_t flank=GPIO_BOTH) {
    // Initialize the pin as input with pull-up, interrupt on falling edge
    if (pcf857x_gpio_init_int(&pcf857x_dev, pin, pullup ? GPIO_IN_PU : GPIO_IN, flank, _gpio_cb, (void *)pin) != PCF857X_OK) {
        std::cerr << "Error: init PCF857x pin " << pin << " failed" << std::endl;
        return;
    }

    _gpio_events[pin] = _gpio_event_pool.create();
    _gpio_events[pin]->list_node.next = nullptr;
    _gpio_pins[_gpio_events[pin]] = pin;

    // Enable interrupt on that pin
    pcf857x_gpio_irq_enable(&pcf857x_dev, pin);
}

/// Init GPIO expander output pin
static void pcf857x_init_output_pin(int pin, int value) {
    if (pcf857x_gpio_init(&pcf857x_dev, pin, GPIO_OUT) != PCF857X_OK) {
        printf("Error: init PCF857X pin %02i failed\n", pin);
        return;
    }
    pcf857x_gpio_write(&pcf857x_dev, pin, value);
}

/// Check central lift bottom limit switch
static int check_limit_switch_central_lift_bottom() {
    return gpio_read(pin_limit_switch_central_lift_bottom);
}

/// Check central lift top limit switch
static int check_limit_switch_central_lift_top() {
    return gpio_read(pin_limit_switch_central_lift_top);
}

/// Check right arm lift bottom limit switch
static int check_limit_switch_right_arm_lift_bottom() {
    return !pf_pcf857x_gpio_read(pin_limit_switch_right_arm_lift_bottom);
}

/// Check right armlift top limit switch
static int check_limit_switch_right_arm_lift_top() {
    return !pf_pcf857x_gpio_read(pin_limit_switch_right_arm_lift_top);
}

/// Check left arm lift bottom limit switch
static int check_limit_switch_left_arm_lift_bottom() {
    return !pf_pcf857x_gpio_read(pin_limit_switch_left_arm_lift_bottom);
}

/// Check left armlift top limit switch
static int check_limit_switch_left_arm_lift_top() {
    return !pf_pcf857x_gpio_read(pin_limit_switch_left_arm_lift_top);
}

/// Init I2C GPIO Expander
static void _pcf8575_init() {
    // Initialize GPIO expander
    if (pcf857x_init(&pcf857x_dev, &pcf857x_params) != PCF857X_OK) {
        puts("Error: PCF8575 init failed!");
        return;
    }
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
        case pin_limit_switch_central_lift_bottom:
            std::cout << "pin_limit_switch_central_lift_bottom triggered" << std::endl;
            _positional_actuators[Enum::MOTOR_CENTRAL_LIFT]->disable_on_check();
            break;
        case pin_limit_switch_central_lift_top:
            std::cout << "pin_limit_switch_central_lift_top triggered" << std::endl;
            _positional_actuators[Enum::MOTOR_CENTRAL_LIFT]->disable_on_check();
            break;
        case pin_limit_switch_right_arm_lift_bottom:
            std::cout << "pin_limit_switch_right_arm_bottom triggered" << std::endl;
            _positional_actuators[Enum::LXMOTOR_RIGHT_ARM_LIFT]->disable_on_check();
            break;
        case pin_limit_switch_right_arm_lift_top:
            std::cout << "pin_limit_switch_right_arm_top triggered" << std::endl;
            _positional_actuators[Enum::LXMOTOR_RIGHT_ARM_LIFT]->disable_on_check();
            break;
        case pin_limit_switch_left_arm_lift_bottom:
            std::cout << "pin_limit_switch_left_arm_bottom triggered" << std::endl;
            _positional_actuators[Enum::LXMOTOR_LEFT_ARM_LIFT]->disable_on_check();
            break;
        case pin_limit_switch_left_arm_lift_top:
            std::cout << "pin_limit_switch_left_arm_top triggered" << std::endl;
            _positional_actuators[Enum::LXMOTOR_LEFT_ARM_LIFT]->disable_on_check();
            break;
        case pin_sensor_pump_right:
            std::cout << "pin_sensor_pump_right triggered" << std::endl;
            _positional_actuators[Enum::LXMOTOR_RIGHT_ARM_LIFT]->disable();
            break;
        case pin_sensor_pump_left:
            std::cout << "pin_sensor_pump_left triggered" << std::endl;
            _positional_actuators[Enum::LXMOTOR_LEFT_ARM_LIFT]->disable();
            break;
        default:
            std::cout << "INT: external interrupt from pin " << pin << std::endl;
            break;
        }
    }

    return nullptr;
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
    // Half duplex stream that must have been initialized previously
    LxMotor::lx_stream = lx_stream;

    // Init GPIO expander
    _pcf8575_init();

    // Init PWM I2C driver
    _pca9685_init();

    // Init central lift limit switches GPIOs
    init_interruptable_pin(pin_limit_switch_central_lift_top, false, GPIO_RISING);
    init_interruptable_pin(pin_limit_switch_central_lift_bottom, false, GPIO_RISING);

    // Init central lift limit switches GPIOs
    init_interruptable_pin(pin_sensor_pump_right, false, GPIO_RISING);
    init_interruptable_pin(pin_sensor_pump_left, false, GPIO_RISING);

    // Init GPIO expander pins - limit switches
    pcf857x_init_interruptable_pin(pin_limit_switch_left_arm_lift_top, true, GPIO_FALLING);
    pcf857x_init_interruptable_pin(pin_limit_switch_left_arm_lift_bottom, true, GPIO_FALLING);
    pcf857x_init_interruptable_pin(pin_limit_switch_right_arm_lift_top, true, GPIO_FALLING);
    pcf857x_init_interruptable_pin(pin_limit_switch_right_arm_lift_bottom, true, GPIO_FALLING);
    pcf857x_init_interruptable_pin(pin_limit_switch_recal_right, true, GPIO_FALLING);
    pcf857x_init_interruptable_pin(pin_limit_switch_recal_left, true, GPIO_FALLING);
    pcf857x_init_output_pin(pin_led_panels, 0);

    // Motors driver init
    motor_driver_init(ACTUATOR_MOTOR_DRIVER_1);
    _positional_actuators[Enum::MOTOR_CENTRAL_LIFT] = _motors_pool.create(
        Enum::MOTOR_CENTRAL_LIFT,
        GroupEnum::NO_GROUP,
        0,
        ACTUATOR_MOTOR_DRIVER_1,
        actuator_central_lift_motor,
        check_limit_switch_central_lift_top,
        check_limit_switch_central_lift_bottom);
    _positional_actuators[Enum::MOTOR_CONVEYOR_LAUNCHER] = _motors_pool.create(
        Enum::MOTOR_CONVEYOR_LAUNCHER,
        GroupEnum::NO_GROUP,
        0,
        ACTUATOR_MOTOR_DRIVER_1,
        actuator_conveyor_launcher_motor);

    // OnOff init
    _positional_actuators[Enum::ONOFF_LED_PANELS] = _onoff_pool.create(
        Enum::ONOFF_LED_PANELS,
        GroupEnum::NO_GROUP,
        0,
        true,
        1,
        pin_led_panels
    );

    // LxMotor
    _positional_actuators[Enum::LXMOTOR_RIGHT_ARM_LIFT] = _lxmotor_pool.create(
        Enum::LXMOTOR_RIGHT_ARM_LIFT,
        GroupEnum::NO_GROUP,
        0,
        LXServoIDs::LXID_RIGHT_ARM_LIFT,
        check_limit_switch_right_arm_lift_top,
        check_limit_switch_right_arm_lift_bottom
    );
    _positional_actuators[Enum::LXMOTOR_LEFT_ARM_LIFT] = _lxmotor_pool.create(
        Enum::LXMOTOR_LEFT_ARM_LIFT,
        GroupEnum::NO_GROUP,
        0,
        LXServoIDs::LXID_LEFT_ARM_LIFT,
        check_limit_switch_left_arm_lift_top,
        check_limit_switch_left_arm_lift_bottom
    );

    // AnalogServo init
    _positional_actuators[Enum::ANALOGSERVO_CHERRY_ARM] = _analog_servo_pool.create(
        Enum::ANALOGSERVO_CHERRY_ARM,
        GroupEnum::NO_GROUP,
        0,
        PCA9586Channels::CHANNEL_ANALOGSERVO_CHERRY_ARM
    );
    static_cast<AnalogServo*>(_positional_actuators[Enum::ANALOGSERVO_CHERRY_ARM])->add_position(analog_servomotor_cherry_arm_closed);
    static_cast<AnalogServo*>(_positional_actuators[Enum::ANALOGSERVO_CHERRY_ARM])->add_position(analog_servomotor_cherry_arm_deployed);

    _positional_actuators[Enum::ANALOGSERVO_CHERRY_ESC] = _analog_servo_pool.create(
        Enum::ANALOGSERVO_CHERRY_ESC,
        GroupEnum::NO_GROUP,
        0,
        PCA9586Channels::CHANNEL_ANALOGSERVO_CHERRY_ESC
    );
    static_cast<AnalogServo*>(_positional_actuators[Enum::ANALOGSERVO_CHERRY_ESC])->add_position(analog_servomotor_cherry_esc_init_off);
    static_cast<AnalogServo*>(_positional_actuators[Enum::ANALOGSERVO_CHERRY_ESC])->add_position(analog_servomotor_cherry_esc_low);
    static_cast<AnalogServo*>(_positional_actuators[Enum::ANALOGSERVO_CHERRY_ESC])->add_position(analog_servomotor_cherry_esc_middle);
    static_cast<AnalogServo*>(_positional_actuators[Enum::ANALOGSERVO_CHERRY_ESC])->add_position(analog_servomotor_cherry_esc_high);
    static_cast<AnalogServo*>(_positional_actuators[Enum::ANALOGSERVO_CHERRY_ESC])->add_position(analog_servomotor_cherry_esc_max);

    _positional_actuators[Enum::ANALOGSERVO_CHERRY_ESC]->actuate(0);


    _positional_actuators[Enum::ANALOGSERVO_CHERRY_RELEASE] = _analog_servo_pool.create(
        Enum::ANALOGSERVO_CHERRY_RELEASE,
        GroupEnum::NO_GROUP,
        0,
        PCA9586Channels::CHANNEL_ANALOGSERVO_CHERRY_RELEASE
    );
    static_cast<AnalogServo*>(_positional_actuators[Enum::ANALOGSERVO_CHERRY_RELEASE])->add_position(analog_servomotor_cherry_release_down);
    static_cast<AnalogServo*>(_positional_actuators[Enum::ANALOGSERVO_CHERRY_RELEASE])->add_position(analog_servomotor_cherry_release_up);

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
        "Positional acturators timeout thread"
    );
}

PositionalActuator & get(Enum id) {
    return *_positional_actuators[id];
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
