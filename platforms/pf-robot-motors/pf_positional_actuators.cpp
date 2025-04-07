// Copyright (C) 2024 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

#include "pf_actuators.hpp"
#include "pf_positional_actuators.hpp"

#include "Motor.hpp"
#include "PositionalActuator.hpp"

#include "actuators_motors_params.hpp"
#include "platform.hpp"
#include "app_conf.hpp"
#include "motion_control_common/BaseController.hpp"
#include "motion_control_common/BaseControllerEngine.hpp"

#include "etl/map.h"
#include "etl/pool.h"

// RIOT includes
#include <event.h>
#include <ztimer.h>
#include <motor_driver.h>
#include <periph/qdec.h>

namespace cogip {
namespace pf {
namespace actuators {
namespace positional_actuators {

extern "C" {
extern int32_t qdecs_value[QDEC_NUMOF];
}

// Motion control motor driver
static motor_driver_t actuators_motors_driver;

// Actuator state protobuf message
static PB_ActuatorState _pb_actuator_state;

/// Positional actuator timeout thread stack
static char _positional_actuators_timeout_thread_stack[THREAD_STACKSIZE_DEFAULT];
/// Positional actuators timeout thread period (ms)
constexpr uint16_t _positional_actuators_timeout_thread_period_ms = 100;
/// GPIOs handler thread stack
static char _gpio_event_handling_thread_stack[THREAD_STACKSIZE_DEFAULT];

/// Motors memory pool
static etl::pool<Motor, COUNT> _motors_pool;
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

// Motor Lift pose PID controller
static cogip::pid::PID motor_lift_pose_pid(
    motor_lift_pose_pid_kp,
    motor_lift_pose_pid_ki,
    motor_lift_pose_pid_kd,
    motor_lift_pose_pid_integral_limit
    );
// Motor Lift speed PID controller
static cogip::pid::PID motor_lift_speed_pid(
    motor_lift_speed_pid_kp,
    motor_lift_speed_pid_ki,
    motor_lift_speed_pid_kd,
    motor_lift_speed_pid_integral_limit
    );

/// Motor Lift MotorPoseFilterParameters
static cogip::motion_control::MotorPoseFilterParameters motor_lift_pose_filter_parameters(
    motor_lift_threshold,
    motor_lift_max_dec_motor_lift_mm_per_period2
    );
/// Motor Lift PosePIDControllerParameters.
static cogip::motion_control::PosePIDControllerParameters motor_lift_pose_pid_parameters(&motor_lift_pose_pid);
/// Motor Lift SpeedFilterParameters.
static cogip::motion_control::SpeedFilterParameters motor_lift_speed_filter_parameters(
    motor_lift_min_speed_motor_lift_mm_per_period,
    motor_lift_max_speed_motor_lift_mm_per_period,
    motor_lift_max_acc_motor_lift_mm_per_period2,
    false,
    motor_lift_anti_blocking_speed_threshold_per_period,
    motor_lift_anti_blocking_error_threshold_per_period,
    motor_lift_anti_blocking_blocked_cycles_nb_threshold
    );
/// Motor Lift SpeedPIDControllerParameters.
static cogip::motion_control::SpeedPIDControllerParameters motor_lift_speed_pid_parameters(&motor_lift_speed_pid);

/// GPIOs interrupt callback
static void _gpio_cb(void *arg)
{
    (void)arg;
    gpio_t pin = (gpio_t)arg;
    event_post(&_new_gpio_event_queue, _gpio_events[pin]);
}

/// GPIO event handler
static void _gpio_event_handler(event_t *event)
{
    gpio_t pin = _gpio_pins[event];

    switch (pin)
    {
        case pin_top_limit_switch_lift:
            std::cout << "pin_top_limit_switch_lift triggered" << std::endl;
            if (gpio_read(pin_bottom_limit_switch_lift)) {
                ((Motor*)_positional_actuators[(cogip::pf::actuators::Enum)Enum::MOTOR_LIFT])->set_current_pose(178);
                ((Motor*)_positional_actuators[(cogip::pf::actuators::Enum)Enum::MOTOR_LIFT])->actuate_timeout(178, 5);
            }
            break;
        case pin_bottom_limit_switch_lift:
            std::cout << "pin_bottom_limit_switch_lift triggered" << std::endl;
            if (gpio_read(pin_top_limit_switch_lift)) {
                ((Motor*)_positional_actuators[(cogip::pf::actuators::Enum)Enum::MOTOR_LIFT])->set_current_pose(0);
                ((Motor*)_positional_actuators[(cogip::pf::actuators::Enum)Enum::MOTOR_LIFT])->actuate_timeout(0, 5);
            }
            break;
        default:
            std::cout << "INT: external interrupt from pin " << pin << std::endl;
            break;
    }
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
    _gpio_events[pin]->handler = _gpio_event_handler;
    _gpio_pins[_gpio_events[pin]] = pin;
}

/// GPIOs handling thread
static void *_gpio_event_handling_thread(void *args)
{
    (void)args;

    // Initialize GPIOs event queue
    event_queue_init(&_new_gpio_event_queue);

    // Wait for events
    event_loop(&_new_gpio_event_queue);

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
                    ((Motor*)positional_actuator)->actuate(
                        ((Motor*)positional_actuator)->current_pose()
                    );
                }
            }
        }

        // Wait thread period to end
        ztimer_periodic_wakeup(ZTIMER_MSEC, &loop_start_time, _positional_actuators_timeout_thread_period_ms);
    }

    return 0;
}

/// Update current speed from quadrature encoders measure.
static void pf_motor_encoder_read_lift(float &current_speed)
{
    current_speed = (qdec_read_and_reset(MOTOR_LIFT_ID) * QDEC_LIFT_POLARITY) / pulse_per_mm;
}

static void compute_current_speed_and_pose_lift(float &current_speed, float &current_pose)
{
    pf_motor_encoder_read_lift(current_speed);
    current_pose += current_speed;
}

static void pf_motor_drive(const int command, cogip::motion_control::BaseControllerEngine &motor_engine, uint8_t motor_id)
{
    // Limit commands to what the PWM driver can accept as input in the range [INT16_MIN:INT16_MAX].
    // The PWM driver will filter the value to the max PWM resolution defined for the board.
    int16_t filtered_command = (int16_t) std::max(std::min(command, std::numeric_limits<int16_t>::max() / 2),
                                               std::numeric_limits<int16_t>::min() / 2);

    // Apply motor commands
    if (fabs(filtered_command) > actuators_motors_driver.params->pwm_resolution) {
        filtered_command = (fabs(filtered_command)/filtered_command) * actuators_motors_driver.params->pwm_resolution - 1;
    }
    filtered_command = (filtered_command < 0 ? -pwm_minimal : pwm_minimal )
                     + ((filtered_command * (int16_t)(actuators_motors_driver.params->pwm_resolution - pwm_minimal))
                        / (int16_t)actuators_motors_driver.params->pwm_resolution);

    motor_set(&actuators_motors_driver, motor_id, filtered_command);
}

static void pf_motor_drive_lift(const int command, cogip::motion_control::BaseControllerEngine &motor_engine) {
    pf_motor_drive(command, motor_engine, MOTOR_LIFT_ID);
    if (motor_engine.pose_reached() == cogip::motion_control::target_pose_status_t::reached) {
        send_state((cogip::pf::actuators::Enum)Enum::MOTOR_LIFT);
    }
}

void pf_init_motors_sequence(void) {
    // Reset motors origin
    ((Motor*)_positional_actuators[(cogip::pf::actuators::Enum)Enum::MOTOR_LIFT])->set_target_speed(motor_lift_max_init_speed_motor_lift_mm_per_period);
    ((Motor*)_positional_actuators[(cogip::pf::actuators::Enum)Enum::MOTOR_LIFT])->actuate_timeout(200, 25);
    ztimer_sleep(ZTIMER_MSEC, 2500);
    ((Motor*)_positional_actuators[(cogip::pf::actuators::Enum)Enum::MOTOR_LIFT])->set_target_speed(motor_lift_max_speed_motor_lift_mm_per_period);
    ((Motor*)_positional_actuators[(cogip::pf::actuators::Enum)Enum::MOTOR_LIFT])->actuate_timeout(0, 15);

    // Send state
    send_state((cogip::pf::actuators::Enum)Enum::MOTOR_LIFT);
}

void init() {
    // Init drivers
    motor_driver_init(&actuators_motors_driver, &actuators_motors_params);

    // Init qdec peripherals
    int error = qdec_init(QDEC_DEV(MOTOR_LIFT_ID), QDEC_MODE, NULL, NULL);
    if (error) {
        printf("QDEC %u not initialized, error=%d !!!\n", MOTOR_LIFT_ID, error);
    }

    // Init limit switches
    init_interruptable_pin(pin_top_limit_switch_lift, GPIO_IN, GPIO_FALLING);
    init_interruptable_pin(pin_bottom_limit_switch_lift, GPIO_IN, GPIO_FALLING);

    _positional_actuators[(cogip::pf::actuators::Enum)Enum::MOTOR_LIFT] = _motors_pool.create(
        (cogip::pf::actuators::Enum)Enum::MOTOR_LIFT,
        0,
        send_state,
        &actuators_motors_driver,
        MOTOR_LIFT_ID,
        CLEAR_OVERLOAD_PIN,
        pin_bottom_limit_switch_lift,
        pin_top_limit_switch_lift,
        motor_lift_max_speed_motor_lift_mm_per_period,
        &motor_lift_pose_pid_parameters,
        &motor_lift_speed_pid_parameters,
        &motor_lift_pose_filter_parameters,
        &motor_lift_speed_filter_parameters,
        cogip::motion_control::motor_get_speed_and_pose_cb_t::create<compute_current_speed_and_pose_lift>(),
        cogip::motion_control::motor_process_commands_cb_t::create<pf_motor_drive_lift>()
    );

    // Init motor to maximum speed and consider its current position as the reference.
    ((Motor*)_positional_actuators[(cogip::pf::actuators::Enum)Enum::MOTOR_LIFT])->set_current_pose(0);
    ((Motor*)_positional_actuators[(cogip::pf::actuators::Enum)Enum::MOTOR_LIFT])->set_target_speed(motor_lift_max_speed_motor_lift_mm_per_period);
    ((Motor*)_positional_actuators[(cogip::pf::actuators::Enum)Enum::MOTOR_LIFT])->actuate_timeout(0,20);

    if (!gpio_read(pin_top_limit_switch_lift)) {
        ((Motor*)_positional_actuators[(cogip::pf::actuators::Enum)Enum::MOTOR_LIFT])->set_current_pose(0);
        ((Motor*)_positional_actuators[(cogip::pf::actuators::Enum)Enum::MOTOR_LIFT])->actuate_timeout(0, 20);
    }

    // Positional actuators timeout thread
    thread_create(
        _gpio_event_handling_thread_stack,
        sizeof(_gpio_event_handling_thread_stack),
        THREAD_PRIORITY_MAIN - 1,
        THREAD_CREATE_STACKTEST,
        _gpio_event_handling_thread,
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

} // namespace positional_actuators
} // namespace actuators
} // namespace pf
} // namespace cogip
