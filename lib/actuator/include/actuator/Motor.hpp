// Copyright (C) 2024 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup     actuator
/// @{
/// @file
/// @brief       C++ class representing a motor using a motor driver and
/// cascaded PID+filter control.
/// @author      Gilles DOFFE <g.doffe@gmail.com>

#pragma once

// Project includes
#include "actuator/MotorParameters.hpp"
#include "actuator/PositionalActuator.hpp"
// Motion control - Classic DualPID chain
#include "dualpid_meta_controller/DualPIDMetaController.hpp"
#include "motor_engine/MotorEngine.hpp"
#include "motor_pose_filter/MotorPoseFilter.hpp"
#include "motor_pose_filter/MotorPoseFilterParameters.hpp"
#include "pose_pid_controller/PosePIDController.hpp"
#include "pose_pid_controller/PosePIDControllerParameters.hpp"
#include "speed_filter/SpeedFilter.hpp"
#include "speed_filter/SpeedFilterParameters.hpp"
#include "speed_pid_controller/SpeedPIDController.hpp"
#include "speed_pid_controller/SpeedPIDControllerParameters.hpp"
// Motion control - Tracker chain (feedforward + feedback)
#include "motion_control_common/MetaController.hpp"
#include "profile_tracker_controller/ProfileTrackerController.hpp"
#include "profile_tracker_controller/ProfileTrackerControllerParameters.hpp"
#include "tracker_combiner_controller/TrackerCombinerController.hpp"
#include "tracker_combiner_controller/TrackerCombinerControllerParameters.hpp"

namespace cogip {
namespace actuators {
namespace positional_actuators {

/// @brief Motor control mode selection
enum class MotorControlMode {
    DUALPID, ///< Classic cascaded PID chain (MotorPoseFilter → PosePID → SpeedFilter → SpeedPID)
    DUALPID_TRACKER ///< Feedforward + feedback chain (ProfileTracker → PosePID → Combiner →
                    ///< SpeedPID)
};

/// @brief Stream insertion operator for actuator Enum ID.
/// @param os   Output stream.
/// @param id   Actuator Enum ID to insert.
/// @return     Reference to the output stream.
std::ostream& operator<<(std::ostream& os, Enum id);

/// @brief Class representing a motor using a motor driver and cascaded
/// PID+filter control.
/// @details
///   Instantiates and configures all sub-controllers and the control engine
///   according to the parameters passed in @ref MotorParameters.
///
///   Supports two control modes:
///   - DUALPID: Classic cascaded PID chain (reactive, may oscillate)
///   - DUALPID_TRACKER: Feedforward + feedback chain (smoother motion)
class Motor : public PositionalActuator
{
  public:
    /// @brief Construct a Motor from its static parameter set.
    /// @param motor_parameters Reference to a `MotorParameters` in static
    /// storage.
    /// @param mode Control mode (DUALPID or DUALPID_TRACKER)
    explicit Motor(const MotorParameters& motor_parameters,
                   MotorControlMode mode = MotorControlMode::DUALPID);

    /// @brief Perform the initialization sequence (e.g., homing).
    void init();

    /// @brief Disable the motor immediately (brake + stop control engine).
    void disable() override;

    /// @brief Enable the motor.
    void enable() override;

    /// @brief Request a new target distance for the motor.
    /// @param command Desired target distance (in encoder units or mm).
    void actuate(int32_t command) override;

    /// @brief Return the target speed as a percentage of the maximum speed.
    /// @return Target speed (in %) relative to max speed.
    float get_target_speed_percentage() const override;

    /// @brief Set the target speed as a percentage of the maximum speed.
    /// @param percentage Target speed (in %) relative to max speed.
    void set_target_speed_percent(float percentage) override;

    /// @brief Get the current measured distance.
    /// @return Current distance in mm.
    float get_current_distance() const;

    /// @brief Manually override the current distance reading.
    /// @param distance New distance in mm.
    void set_current_distance(float distance);

    /// @brief Get the control mode.
    /// @return The current control mode.
    MotorControlMode control_mode() const
    {
        return control_mode_;
    }

  protected:
    /// Reference to the static parameter set.
    const MotorParameters& params_;

    /// Control mode (DUALPID or DUALPID_TRACKER)
    MotorControlMode control_mode_;

    // =========================================================================
    // Classic DualPID chain controllers
    // =========================================================================

    /// Cascade controller combining pose and speed control (classic chain).
    motion_control::DualPIDMetaController dualpid_meta_controller_;

    /// Position (pose) PID controller (classic chain).
    motion_control::PosePIDController distance_controller_;

    /// Speed PID controller (classic chain).
    motion_control::SpeedPIDController speed_controller_;

    /// Filter for pose (distance) measurement (classic chain).
    motion_control::MotorPoseFilter motor_distance_filter_;

    /// Filter for speed/acceleration measurement (classic chain).
    motion_control::SpeedFilter speed_filter_;

    // =========================================================================
    // Tracker chain controllers (feedforward + feedback)
    // =========================================================================

    /// Meta controller for tracker chain.
    motion_control::MetaController<> tracker_meta_controller_;

    /// Profile tracker controller - generates trapezoidal velocity profile.
    motion_control::ProfileTrackerController profile_tracker_controller_;

    /// Pose PID controller for tracking error correction (tracker chain).
    motion_control::PosePIDController tracker_pose_controller_;

    /// Tracker combiner - adds feedforward + feedback.
    motion_control::TrackerCombinerController tracker_combiner_controller_;

    /// Speed PID controller (tracker chain).
    motion_control::SpeedPIDController tracker_speed_controller_;

    /// Filter for pose (distance) measurement (tracker chain).
    motion_control::MotorPoseFilter tracker_motor_distance_filter_;

    // =========================================================================
    // Common components
    // =========================================================================

    /// Threaded control engine managing the control loop.
    motion_control::MotorEngine motor_engine_;
};

} // namespace positional_actuators
} // namespace actuators
} // namespace cogip

/// @}
