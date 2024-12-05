// Copyright (C) 2024 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @brief       Differential drive controller
/// @author      Mathis Lécrivain <lecrivain.mathis@gmail.com>

#pragma once

namespace cogip {

namespace drive_controller {

class DifferentialDriveControllerParameters {
public:
    /// @brief Construct a new DifferentialDriveControllerParameters object
    /// @param left_wheel_diameter left wheel diameter value (mm)
    /// @param right_wheel_diameter right wheel diameter value (mm)
    /// @param track_width Track width value (mm)
    /// @param left_motor_constant left motor constant (no unit)
    /// @param right_motor_constant right motor constant (no unit)
    /// @param max_speed_percentage max speed ration to send to motors (%) in range [0;100]
    /// @param loop_period regulation loop period (ms)
    DifferentialDriveControllerParameters(double left_wheel_diameter,
                                          double right_wheel_diameter,
                                          double track_width,
                                          double left_motor_constant,
                                          double right_motor_constant,
                                          double max_speed_percentage,
                                          double loop_period)
                                          : left_wheel_diameter_(left_wheel_diameter),
                                            right_wheel_diameter_(right_wheel_diameter),
                                            track_width_(track_width),
                                            left_motor_constant_(left_motor_constant),
                                            right_motor_constant_(right_motor_constant),
                                            max_speed_percentage_(max_speed_percentage),
                                            loop_period_(loop_period) {};

    /// @brief Set the left motor wheel diameter dimension
    /// @param wheel_diameter Wheel diameter value (mm)
    void set_left_wheel_diameter(double wheel_diameter) { left_wheel_diameter_ = wheel_diameter; }

    /// @brief Return left motor wheel diameter value in mm.
    /// @return double Wheel diameter value (mm)
    double left_wheel_diameter_mm() const { return left_wheel_diameter_; }

    /// @brief Set the right wheel motor diameter dimension
    /// @param wheel_diameter Wheel diameter value (mm)
    void set_right_wheel_diameter(double wheel_diameter) { right_wheel_diameter_ = wheel_diameter; }

    /// @brief Return right motor wheel diameter value in mm.
    /// @return double Wheel diameter value (mm)
    double right_wheel_diameter_mm() const { return right_wheel_diameter_; }

    /// @brief Set the left motor constant value.
    ///        This value needs to be defined by using the motors characteristics from the datasheet.
    ///        This value is useful to convert linear velocity in mm/s to a speed ration between 0 to 100%.
    ///        This speed ratio need to by applied to the max motor voltage in order to get the voltage needed
    ///        to theoretically reach the desired speed.
    ///
    /// @note motor_constant = ((60 * Reduction Ratio / velocity Constant (RPM/V)) / Motor Voltage (V)) * 100
    /// @param motor_constant motor constant (no unit)
    void set_left_motor_constant(double motor_constant) { left_motor_constant_ = motor_constant; }

    /// @brief Return left motor constant value
    /// @return double motor constant (no unit)
    double left_motor_constant() const { return left_motor_constant_; }

    /// @brief Set the right wheel motor constant value.
    ///        This value needs to be defined by using the motors characteristics from the datasheet.
    ///        This value is useful to convert linear velocity in mm/s to a speed ration between 0 to 100%.
    ///        This speed ratio need to by applied to the max motor voltage in order to get the voltage needed
    ///        to theoretically reach the desired speed.
    ///
    /// @note motor_constant = ((60 * Reduction Ratio / velocity Constant (RPM/V)) / Motor Voltage (V)) * 100
    /// @param motor_constant motor constant (no unit)
    void set_right_motor_constant(double motor_constant) { right_motor_constant_ = motor_constant; }

    /// @brief Return right motor constant value
    /// @return double Wheel motor constant (no unit)
    double right_motor_constant() const { return right_motor_constant_; }

    /// @brief Set the axle track dimension
    /// @param track_width Track width value (mm)
    void set_track_width(double track_width) { track_width_ = track_width; }

    /// @brief Get the axle track value
    /// @return double Track width value (mm)
    double track_width_mm() const { return track_width_; }

    /// @brief Set the max speed ratio in percent
    /// @param max speed ratio (%)
    void set_max_speed_percentage(double max) { max_speed_percentage_ = max; }

    /// @brief get the max speed ratio in percent
    /// @return double speed ratio (%)
    double max_speed_percentage() const { return max_speed_percentage_; }

    /// @brief Set the loop period in milliseconds
    /// @param period loop period (ms)
    void set_loop_period(double period) { loop_period_ = period; }

    /// @brief Get the axle track value
    /// @return double Track width value (mm)
    double loop_period_ms() const { return loop_period_; }

private:
    double left_wheel_diameter_;
    double right_wheel_diameter_;
    double track_width_;
    double left_motor_constant_;
    double right_motor_constant_;
    double max_speed_percentage_;
    double loop_period_;
};

} // namespace odometer

} // namespace cogip
