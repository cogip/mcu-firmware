// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

#pragma once

namespace cogip {

namespace localization {

class LocalizationDifferentialParameters {
public:
    /// @brief Construct a new Odometry Params object
    /// @param left_wheel_diameter left wheel diameter value (mm)
    /// @param right_wheel_diameter right wheel diameter value (mm)
    /// @param track_width Track width value (mm)
    /// @param left_polarity left wheel polarity no unit, 1 or -1
    /// @param right_polarity riht wheel polarity no unit, 1 or -1
    LocalizationDifferentialParameters(float left_wheel_diameter,
    float right_wheel_diameter, float track_width, float left_polarity, float right_polarity) : left_wheel_diameter_(left_wheel_diameter), right_wheel_diameter_(right_wheel_diameter), track_width_(track_width), left_polarity_(left_polarity), right_polarity_(right_polarity) {};

    /// @brief Set the left encoder wheel diameter dimension
    /// @param wheel_diameter Wheel diameter value (mm)
    void set_left_wheel_diameter(float wheel_diameter) { left_wheel_diameter_ = wheel_diameter; }

    /// @brief Return left encoder wheel diameter value in mm.
    /// @return float Wheel diameter value (mm)
    float left_wheel_diameter_mm() const { return left_wheel_diameter_; }

    /// @brief Set the right wheel encoder diameter dimension
    /// @param wheel_diameter Wheel diameter value (mm)
    void set_right_wheel_diameter(float wheel_diameter) { right_wheel_diameter_ = wheel_diameter; }

    /// @brief Return right encoder wheel diameter value in mm.
    /// @return float Wheel diameter value (mm)
    float right_wheel_diameter_mm() const { return right_wheel_diameter_; }

    /// @brief Set the axle track dimension
    /// @param track_width Track width value (mm)
    void set_track_width(float track_width) { track_width_ = track_width; }

    /// @brief Get the axle track value
    /// @return float Track width value (mm)
    float track_width_mm() const { return track_width_; }

    /// @brief Set the left encoder polarity
    /// @param polarity Encoder polarity. -1 or 1.
    void set_left_polarity(float polarity) { left_polarity_ = polarity; }

    /// @brief Get the left encoder polarity
    /// @return float Encoder polarity. -1 or 1
    float left_polarity() const { return left_polarity_; }

    /// @brief Set the right encoder polarity
    /// @param polarity Encoder polarity. -1 or 1.
    void set_right_polarity(float polarity) { right_polarity_ = polarity; }

    /// @brief Get the right encoder polarity
    /// @return float Encoder polarity. -1 or 1
    float right_polarity() const { return right_polarity_; }

private:
    float left_wheel_diameter_;
    float right_wheel_diameter_;
    float track_width_;
    float left_polarity_;
    float right_polarity_;
};

} // namespace localization

} // namespace cogip
