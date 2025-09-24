// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @brief       Differential drive controller parameters
/// @author      Mathis Lécrivain <lecrivain.mathis@gmail.com>

#pragma once

namespace cogip {

namespace drive_controller {

class DifferentialDriveControllerParameters
{
  public:
    /// @brief Construct a new DifferentialDriveControllerParameters object
    /// @param left_wheel_diameter left wheel diameter value (mm)
    /// @param right_wheel_diameter right wheel diameter value (mm)
    /// @param track_width Track width value (mm)
    /// @param left_motor_constant left motor constant (no unit)
    /// @param right_motor_constant right motor constant (no unit)
    /// @param min_speed_percentage min speed ratio to send to motors (%) in range
    /// [0;100]
    /// @param max_speed_percentage max speed ratio to send to motors (%) in range
    /// [0;100]
    /// @param loop_period regulation loop period (ms)
    DifferentialDriveControllerParameters(float left_wheel_diameter, float right_wheel_diameter,
                                          float track_width, float left_motor_constant,
                                          float right_motor_constant, float min_speed_percentage,
                                          float max_speed_percentage, float loop_period)
        : left_wheel_diameter_(left_wheel_diameter), right_wheel_diameter_(right_wheel_diameter),
          track_width_(track_width), left_motor_constant_(left_motor_constant),
          right_motor_constant_(right_motor_constant), min_speed_percentage_(min_speed_percentage),
          max_speed_percentage_(max_speed_percentage), loop_period_(loop_period){};

    /// @brief Set the left motor wheel diameter dimension
    /// @param wheel_diameter Wheel diameter value (mm)
    void set_left_wheel_diameter(float wheel_diameter)
    {
        left_wheel_diameter_ = wheel_diameter;
    }

    /// @brief Return left motor wheel diameter value in mm.
    /// @return float Wheel diameter value (mm)
    float left_wheel_diameter_mm() const
    {
        return left_wheel_diameter_;
    }

    /// @brief Set the right wheel motor diameter dimension
    /// @param wheel_diameter Wheel diameter value (mm)
    void set_right_wheel_diameter(float wheel_diameter)
    {
        right_wheel_diameter_ = wheel_diameter;
    }

    /// @brief Return right motor wheel diameter value in mm.
    /// @return float Wheel diameter value (mm)
    float right_wheel_diameter_mm() const
    {
        return right_wheel_diameter_;
    }

    /// @brief Set the left motor constant value.
    ///        This value needs to be defined by using the motors characteristics
    ///        from the datasheet. This value is useful to convert linear velocity
    ///        in mm/s to a speed ratio between 0 to 100%. This speed ratio need
    ///        to be applied to the max motor voltage in order to get the voltage
    ///        needed to theoretically reach the desired speed.
    ///
    /// @note motor_constant = ((60 * Reduction Ratio / velocity Constant (RPM/V))
    /// / Motor Voltage (V)) * 100
    /// @param motor_constant motor constant (no unit)
    void set_left_motor_constant(float motor_constant)
    {
        left_motor_constant_ = motor_constant;
    }

    /// @brief Return left motor constant value
    /// @return float motor constant (no unit)
    float left_motor_constant() const
    {
        return left_motor_constant_;
    }

    /// @brief Set the right wheel motor constant value.
    ///        This value needs to be defined by using the motors characteristics
    ///        from the datasheet. This value is useful to convert linear velocity
    ///        in mm/s to a speed ratio between 0 to 100%. This speed ratio need
    ///        to by applied to the max motor voltage in order to get the voltage
    ///        needed to theoretically reach the desired speed.
    ///
    /// @note motor_constant = ((60 * Reduction Ratio / velocity Constant (RPM/V))
    /// / Motor Voltage (V)) * 100
    /// @param motor_constant motor constant (no unit)
    void set_right_motor_constant(float motor_constant)
    {
        right_motor_constant_ = motor_constant;
    }

    /// @brief Return right motor constant value
    /// @return float Wheel motor constant (no unit)
    float right_motor_constant() const
    {
        return right_motor_constant_;
    }

    /// @brief Set the axle track dimension
    /// @param track_width Track width value (mm)
    void set_track_width(float track_width)
    {
        track_width_ = track_width;
    }

    /// @brief Get the axle track value
    /// @return float Track width value (mm)
    float track_width_mm() const
    {
        return track_width_;
    }

    /// @brief Set the min speed ratio in percent
    /// @param min speed ratio (%)
    void set_min_speed_percentage(float min)
    {
        min_speed_percentage_ = min;
    }

    /// @brief get the min speed ratio in percent
    /// @return float speed ratio (%)
    float min_speed_percentage()
    {
        return min_speed_percentage_;
    }

    /// @brief Set the max speed ratio in percent
    /// @param max speed ratio (%)
    void set_max_speed_percentage(float max)
    {
        max_speed_percentage_ = max;
    }

    /// @brief get the max speed ratio in percent
    /// @return float speed ratio (%)
    float max_speed_percentage() const
    {
        return max_speed_percentage_;
    }

    /// @brief Set the loop period in milliseconds
    /// @param period loop period (ms)
    void set_loop_period(float period)
    {
        loop_period_ = period;
    }

    /// @brief Get the axle track value
    /// @return float Track width value (mm)
    float loop_period_ms() const
    {
        return loop_period_;
    }

  private:
    float left_wheel_diameter_;
    float right_wheel_diameter_;
    float track_width_;
    float left_motor_constant_;
    float right_motor_constant_;
    float min_speed_percentage_;
    float max_speed_percentage_;
    float loop_period_;
};

} // namespace drive_controller

} // namespace cogip
