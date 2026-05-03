// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

#include "log.h"
#include <inttypes.h>
#include <periph/gpio.h>
#include <ztimer.h>

#include "actuator/Lift.hpp"
#include "actuator/LiftsLimitSwitchesManager.hpp"

namespace cogip {
namespace actuators {
namespace positional_actuators {

Lift::Lift(const LiftParameters& params)
    : Motor(params.motor_params,
            // Use DUALPID_TRACKER mode if profile_tracker_parameters is set
            params.motor_params.profile_tracker_parameters != nullptr
                ? MotorControlMode::DUALPID_TRACKER
                : MotorControlMode::DUALPID),
      params_(params)
{
}

void Lift::init()
{
    // Invalidate any stale command from a previous run: init can be
    // re-triggered over CAN, and the "else" branch below (already at
    // lower limit) never calls actuate(), so last_command_ would
    // otherwise keep a value from a previous session and cause the
    // next matching actuate() to be wrongly skipped.
    last_command_ = INT32_MIN;

    int ret =
        LiftsLimitSwitchesManager::instance().register_gpio(params_.lower_limit_switch_pin, this);
    if (ret) {
        LOG_ERROR("Lower switch pin for lift %" PRIu8 " registering failed\n",
                  static_cast<uint8_t>(id_));
    }
    ret = LiftsLimitSwitchesManager::instance().register_gpio(params_.upper_limit_switch_pin, this);
    if (ret) {
        LOG_ERROR("Upper switch pin for lift %" PRIu8 " registering failed\n",
                  static_cast<uint8_t>(id_));
    }

    // Homing: move down to calibrate zero position
    set_target_speed_percent(params_.init_speed_percentage);

    int lower_state = gpio_read(params_.lower_limit_switch_pin);
    LOG_INFO("Lift init: lower_switch=%d\n", lower_state);

    constexpr uint32_t init_timeout_ms = 2000;

    if (!lower_state) {
        // Lock the homing mutex up front; at_lower_limit() unlocks it on the
        // rising edge of the lower switch, so the wait below exits the
        // moment the descent is physically done.
        mutex_lock(&initializing_);

        // Force current position to upper limit so the motor moves down
        set_current_distance(params_.upper_limit_mm);
        LOG_INFO("Moving to lower limit (%d mm)...\n", static_cast<int>(params_.lower_limit_mm));
        actuate_timeout(params_.lower_limit_mm, init_timeout_ms);

        // Block until the ISR signals that the switch was hit, with a hard
        // cap of init_timeout_ms so a stuck lift never wedges the CAN
        // handler thread for longer than the homing timeout.
        int wait_ret = ztimer_mutex_lock_timeout(ZTIMER_MSEC, &initializing_, init_timeout_ms);
        if (wait_ret < 0) {
            // Lower limit was not reached: cut the motor to prevent
            // violent movement on power restore (e.g. after emergency
            // stop). initializing_ is still locked from the up-front
            // mutex_lock; release it below.
            LOG_INFO("Init timeout, disabling motor\n");
            disable();
        }
        // On success, the ISR's mutex_unlock + our lock_timeout dance left
        // the mutex re-locked on our side; on timeout, it is still locked
        // from the up-front lock. Either way, leave it unlocked so the
        // next init() can lock it again.
        mutex_unlock(&initializing_);
        LOG_INFO("Lower limit reached or timeout\n");
    } else {
        LOG_INFO("Already at lower limit, setting distance to %d mm\n",
                 static_cast<int>(params_.lower_limit_mm));
        set_current_distance(params_.lower_limit_mm);
    }

    LOG_INFO("Lift init complete\n");
}

void Lift::stop()
{
    actuate(get_current_distance());
}

void Lift::actuate(int32_t command)
{
    LOG_INFO("Move lift to command %" PRIi32 "\n", command);
    // clamp within mechanical bounds
    const int32_t clamped = (command < params_.lower_limit_mm)   ? params_.lower_limit_mm
                            : (command > params_.upper_limit_mm) ? params_.upper_limit_mm
                                                                 : command;

    if (clamped == last_command_) {
        LOG_INFO("Lift already at command %" PRIi32 ", skipping\n", clamped);
        return;
    }

    LOG_INFO("Move lift to clamped command %" PRIi32 "\n", clamped);

    last_command_ = clamped;
    Motor::actuate(clamped);
}

void Lift::at_limits(gpio_t pin)
{
    if (pin == params_.lower_limit_switch_pin) {
        at_lower_limit();
        return;
    } else if (pin == params_.upper_limit_switch_pin) {
        at_upper_limit();
        return;
    } else {
        LOG_ERROR("Pin not handled by this lift\n");
    }
}

void Lift::on_state_change(motion_control::target_pose_status_t state)
{
    if (state == motion_control::target_pose_status_t::blocked ||
        state == motion_control::target_pose_status_t::timeout) {
        last_command_ = INT32_MIN;
    }
    Motor::on_state_change(state);
}

void Lift::at_lower_limit()
{
    // Only react when switch is pressed (reads 1 with active-high logic)
    if (gpio_read(params_.lower_limit_switch_pin)) {
        LOG_INFO("Lower limit switch pressed\n");
        // Always wake init() out of its homing wait, even on a spurious
        // press, so the homing path never wedges on a glitch. Safe to
        // call when no init is in flight: unlocking an already-unlocked
        // RIOT mutex is a no-op.
        mutex_unlock(&initializing_);

        // The switch pressing is a definitive mechanical reference: zero
        // the odometer against it unconditionally. This corrects any
        // drift from controller overshoot, belt slip, or accumulated
        // encoder error, regardless of what the host had asked for.
        set_current_distance(params_.lower_limit_mm);

        // We just zeroed current to lower_limit_mm, so the test
        // collapses to a pure intent check on last_command_: if the
        // host commanded a target above the switch, the press is a
        // glitch on our way up (mechanical bounce on departure) or a
        // belt-slip detection, and we must not interrupt the motion.
        // Otherwise the host wanted to be at the switch and we treat
        // the press as genuine end-of-travel.
        if (last_command_ > params_.lower_limit_mm) {
            LOG_INFO("Lower switch pressed with target above (last_cmd=%" PRIi32 "), resync only\n",
                     last_command_);
            return;
        }

        // The switch pressing IS the definitive "reached" signal. Emit
        // it before disable() cuts the engine, otherwise the engine
        // will never run another tick to fire pose_reached_cb and the
        // host would never hear that the target was reached.
        on_state_change(motion_control::target_pose_status_t::reached);
        disable();
        // Invalidate last_command_ so the next actuate() with the same
        // target is not skipped and can re-enable the motor.
        last_command_ = INT32_MIN;
    } else {
        LOG_INFO("Lower limit switch released\n");
    }
}

void Lift::at_upper_limit()
{
    // Only react when switch is pressed (reads 1 with active-high logic)
    if (gpio_read(params_.upper_limit_switch_pin)) {
        LOG_INFO("Upper limit switch pressed\n");

        // Decide based on motion direction, not on the exact target. A
        // belt slip can overshoot us into the upper switch even when
        // the requested command was below upper_limit_mm: in that case
        // the switch press is still a genuine end-of-travel event we
        // must honor (mechanical safety wins).
        const float current = get_current_distance();
        const bool moving_down = static_cast<float>(last_command_) < current;
        if (moving_down) {
            // Switch press while we are descending: spurious
            // (mechanical bounce, manual interaction). Latching the
            // brake chain here would silently freeze the in-flight
            // downward motion.
            LOG_WARNING("Upper limit pressed while descending (cur=%.1f, "
                        "last_cmd=%" PRIi32 "), ignoring\n",
                        static_cast<double>(current), last_command_);
            return;
        }
        motor_engine_.set_timeout_enable(false);
        // The switch pressing IS the definitive "reached" signal. Emit
        // it now: the brake chain takes over and does not drive the
        // pose loop, so MotorEngine::process_outputs() cannot fire
        // pose_reached_cb on its own from here on.
        on_state_change(motion_control::target_pose_status_t::reached);
        // Latch the engine brake chain: the zero-speed-order + speed PID
        // chain actively drives the motor toward zero speed, holding the
        // lift against gravity without needing the full pose loop.
        // Motor::actuate() releases the brake explicitly on a new motion
        // request.
        motor_engine_.set_brake(true);
        // Invalidate last_command_ so the next actuate() with the same
        // target is not skipped and can release the brake.
        last_command_ = INT32_MIN;
    } else {
        LOG_INFO("Upper limit switch released\n");
    }
}

} // namespace positional_actuators
} // namespace actuators
} // namespace cogip
