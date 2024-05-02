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

// Positional actuator protobuf message
static PB_PositionalActuator _pb_positional_actuator;

/// Positional actuator timeout thread stack
static char _positional_actuators_timeout_thread_stack[THREAD_STACKSIZE_DEFAULT];
/// Positional actuators timeout thread period (ms)
constexpr uint16_t _positional_actuators_timeout_thread_period_ms = 100;
/// GPIOs handler thread stack
static char _gpio_handling_thread_stack[THREAD_STACKSIZE_DEFAULT];

/// Motors memory pool
static etl::pool<Motor, COUNT> _motors_pool;
/// Positional actuators map
static etl::map<Enum, PositionalActuator *, 4*COUNT> _positional_actuators;

/// GPIOs event pool
static etl::pool<event_t, 20> _gpio_event_pool;
/// GPIOs event map
static etl::map<gpio_t, event_t *, 20> _gpio_events;

/// GPIO event queue
static event_queue_t _new_gpio_event_queue;

// Motor bottom lift pose PID controller
static cogip::pid::PID motor_bottom_lift_pose_pid(
    motor_lift_pose_pid_kp,
    motor_lift_pose_pid_ki,
    motor_lift_pose_pid_kd,
    motor_lift_pose_pid_integral_limit
    );
// Motor bottom lift speed PID controller
static cogip::pid::PID motor_bottom_lift_speed_pid(
    motor_lift_speed_pid_kp,
    motor_lift_speed_pid_ki,
    motor_lift_speed_pid_kd,
    motor_lift_speed_pid_integral_limit
    );
// Motor top lift pose PID controller
static cogip::pid::PID motor_top_lift_pose_pid(
    motor_lift_pose_pid_kp,
    motor_lift_pose_pid_ki,
    motor_lift_pose_pid_kd,
    motor_lift_pose_pid_integral_limit
    );
// Motor top lift speed PID controller
static cogip::pid::PID motor_top_lift_speed_pid(
    motor_lift_speed_pid_kp,
    motor_lift_speed_pid_ki,
    motor_lift_speed_pid_kd,
    motor_lift_speed_pid_integral_limit
    );

/// Motor Bottom lift MotorPoseFilterParameters
static cogip::motion_control::MotorPoseFilterParameters motor_bottom_lift_pose_filter_parameters(
    motor_lift_threshold,
    motor_lift_max_dec_motor_lift_mm_per_period2
    );
/// Motor Top lift MotorPoseFilterParameters
static cogip::motion_control::MotorPoseFilterParameters motor_top_lift_pose_filter_parameters(
    motor_lift_threshold,
    motor_lift_max_dec_motor_lift_mm_per_period2
    );
/// Motor Bottom Lift PosePIDControllerParameters.
static cogip::motion_control::PosePIDControllerParameters motor_bottom_lift_pose_pid_parameters(&motor_bottom_lift_pose_pid);
/// Motor Top Lift PosePIDControllerParameters.
static cogip::motion_control::PosePIDControllerParameters motor_top_lift_pose_pid_parameters(&motor_top_lift_pose_pid);
/// Motor Bottom Lift SpeedFilterParameters.
static cogip::motion_control::SpeedFilterParameters motor_bottom_lift_speed_filter_parameters(
    motor_lift_min_speed_motor_lift_mm_per_period,
    motor_lift_max_speed_motor_lift_mm_per_period,
    motor_lift_max_acc_motor_lift_mm_per_period2,
    true,
    motor_lift_anti_blocking_speed_threshold_per_period,
    motor_lift_anti_blocking_error_threshold_per_period,
    motor_lift_anti_blocking_blocked_cycles_nb_threshold
    );
/// Motor Top Lift SpeedFilterParameters.
static cogip::motion_control::SpeedFilterParameters motor_top_lift_speed_filter_parameters(
    motor_lift_min_speed_motor_lift_mm_per_period,
    motor_lift_max_speed_motor_lift_mm_per_period,
    motor_lift_max_acc_motor_lift_mm_per_period2,
    true,
    motor_lift_anti_blocking_speed_threshold_per_period,
    motor_lift_anti_blocking_error_threshold_per_period,
    motor_lift_anti_blocking_blocked_cycles_nb_threshold
    );
/// Motor Bottom Lift SpeedPIDControllerParameters.
static cogip::motion_control::SpeedPIDControllerParameters motor_bottom_lift_speed_pid_parameters(&motor_bottom_lift_speed_pid);
/// Motor Top Lift SpeedPIDControllerParameters.
static cogip::motion_control::SpeedPIDControllerParameters motor_top_lift_speed_pid_parameters(&motor_top_lift_speed_pid);

/// GPIOs interrupt callback
static void _gpio_cb(void *arg)
{
    (void)arg;
    gpio_t pin = (gpio_t)arg;
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
                }
            }
        }

        // Wait thread period to end
        ztimer_periodic_wakeup(ZTIMER_MSEC, &loop_start_time, _positional_actuators_timeout_thread_period_ms);
    }

    return 0;
}

/// Update current speed from quadrature encoders measure.
static void pf_motor_encoder_read_bottom_lift(double &current_speed)
{
    current_speed = (qdec_read_and_reset(MOTOR_BOTTOM_LIFT_ID) * QDEC_BOTTOM_LIFT_POLARITY) / pulse_per_mm;
}

/// Update current speed from quadrature encoders measure.
static void pf_motor_encoder_read_top_lift(double &current_speed)
{
    current_speed = (qdec_read_and_reset(MOTOR_TOP_LIFT_ID) * QDEC_TOP_LIFT_POLARITY) / pulse_per_mm;
}

static void compute_current_speed_and_pose_bottom_lift(double &current_speed, double &current_pose)
{
    pf_motor_encoder_read_bottom_lift(current_speed);
    current_pose += current_speed;
}

static void compute_current_speed_and_pose_top_lift(double &current_speed, double &current_pose)
{
    pf_motor_encoder_read_top_lift(current_speed);
    current_pose += current_speed;
}

static void pf_motor_drive(const int command, cogip::motion_control::BaseControllerEngine &motor_engine, uint8_t motor_id)
{
    if ((motor_engine.pose_reached() == cogip::motion_control::target_pose_status_t::reached)
        || (motor_engine.pose_reached() == cogip::motion_control::target_pose_status_t::blocked)) {
        motor_brake(&actuators_motors_driver, motor_id);
        motor_engine.disable();
        return;
    }

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

static void pf_motor_drive_bottom_lift(const int command, cogip::motion_control::BaseControllerEngine &motor_engine) {
    pf_motor_drive(command, motor_engine, MOTOR_BOTTOM_LIFT_ID);
}

static void pf_motor_drive_top_lift(const int command, cogip::motion_control::BaseControllerEngine &motor_engine) {
    pf_motor_drive(command, motor_engine, MOTOR_TOP_LIFT_ID);
}

void pf_init_motors_sequence(void) {
    // Move motors until blocked to initialize them.
    // WARNING: This implies to configure Rsense on motor board correctly.
    //          Wrong Rsense configuration could damage the motor if software anti blocking fails.

    // 1. Slow down motors speed
    ((Motor*)_positional_actuators[Enum::MOTOR_BOTTOM_LIFT])->set_target_speed(motor_lift_max_init_speed_motor_lift_mm_per_period);
    ((Motor*)_positional_actuators[Enum::MOTOR_TOP_LIFT])->set_target_speed(motor_lift_max_init_speed_motor_lift_mm_per_period);

    // 2. Down motors until hardware/software anti-blocking or timeout stops the motor
    _positional_actuators[Enum::MOTOR_BOTTOM_LIFT]->actuate_timeout(INT16_MIN, default_timeout_period_motor_bottom_lift*3);
    ztimer_sleep(ZTIMER_MSEC, 200);
    _positional_actuators[Enum::MOTOR_TOP_LIFT]->actuate_timeout(INT16_MIN, default_timeout_period_motor_top_lift*3);
    ztimer_sleep(ZTIMER_MSEC, default_timeout_period_motor_top_lift * _positional_actuators_timeout_thread_period_ms*3);

    // 3. Restore maximum speed
    ((Motor*)_positional_actuators[Enum::MOTOR_BOTTOM_LIFT])->set_target_speed(motor_lift_max_speed_motor_lift_mm_per_period);
    ((Motor*)_positional_actuators[Enum::MOTOR_TOP_LIFT])->set_target_speed(motor_lift_max_speed_motor_lift_mm_per_period);

    // 4. Reset motors origin
    ((Motor*)_positional_actuators[Enum::MOTOR_BOTTOM_LIFT])->set_current_pose(0);
    ((Motor*)_positional_actuators[Enum::MOTOR_TOP_LIFT])->set_current_pose(0);

    // 5. Reach initial pose
    _positional_actuators[Enum::MOTOR_TOP_LIFT]->actuate_timeout(motor_top_lift_initial_pose, default_timeout_period_motor_top_lift);
    ztimer_sleep(ZTIMER_MSEC, 200);
    _positional_actuators[Enum::MOTOR_BOTTOM_LIFT]->actuate_timeout(motor_bottom_lift_initial_pose, default_timeout_period_motor_bottom_lift);
    ztimer_sleep(ZTIMER_MSEC, default_timeout_period_motor_bottom_lift * _positional_actuators_timeout_thread_period_ms);

    send_state(Enum::MOTOR_BOTTOM_LIFT);
    send_state(Enum::MOTOR_TOP_LIFT);
}

void init() {
    // Init drivers
    motor_driver_init(&actuators_motors_driver, &actuators_motors_params);

    // Init qdec peripherals
    int error = qdec_init(QDEC_DEV(MOTOR_BOTTOM_LIFT_ID), QDEC_MODE, NULL, NULL);
    if (error) {
        printf("QDEC %u not initialized, error=%d !!!\n", MOTOR_BOTTOM_LIFT_ID, error);
    }
    error = qdec_init(QDEC_DEV(MOTOR_TOP_LIFT_ID), QDEC_MODE, NULL, NULL);
    if (error) {
        printf("QDEC %u not initialized, error=%d !!!\n", MOTOR_TOP_LIFT_ID, error);
    }

    _positional_actuators[Enum::MOTOR_BOTTOM_LIFT] = _motors_pool.create(
        Enum::MOTOR_BOTTOM_LIFT,
        GroupEnum::NO_GROUP,
        0,
        0,
        &actuators_motors_driver,
        MOTOR_BOTTOM_LIFT_ID,
        CLEAR_OVERLOAD_PIN,
        motor_lift_max_speed_motor_lift_mm_per_period,
        &motor_bottom_lift_pose_pid_parameters,
        &motor_bottom_lift_speed_pid_parameters,
        &motor_bottom_lift_pose_filter_parameters,
        &motor_bottom_lift_speed_filter_parameters,
        cogip::motion_control::motor_get_speed_and_pose_cb_t::create<compute_current_speed_and_pose_bottom_lift>(),
        cogip::motion_control::motor_process_commands_cb_t::create<pf_motor_drive_bottom_lift>()
    );
    _positional_actuators[Enum::MOTOR_TOP_LIFT] = _motors_pool.create(
        Enum::MOTOR_TOP_LIFT,
        GroupEnum::NO_GROUP,
        0,
        0,
        &actuators_motors_driver,
        MOTOR_TOP_LIFT_ID,
        CLEAR_OVERLOAD_PIN,
        motor_lift_max_speed_motor_lift_mm_per_period,
        &motor_top_lift_pose_pid_parameters,
        &motor_top_lift_speed_pid_parameters,
        &motor_top_lift_pose_filter_parameters,
        &motor_top_lift_speed_filter_parameters,
        cogip::motion_control::motor_get_speed_and_pose_cb_t::create<compute_current_speed_and_pose_top_lift>(),
        cogip::motion_control::motor_process_commands_cb_t::create<pf_motor_drive_top_lift>()
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

bool contains(Enum id) {
    return _positional_actuators.contains(id);
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

} // namespace positional_actuators
} // namespace actuators
} // namespace pf
} // namespace cogip
