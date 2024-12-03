#pragma once

namespace cogip {

namespace odometer {

class OdometerDifferentialParameters {
public:
    /// @brief Construct a new Odometry Params object
    /// @param left_wheel_diameter left wheel diameter value (mm)
    /// @param right_wheel_diameter right wheel diameter value (mm)
    /// @param track_width Track width value (mm)
    /// @param left_polarity left wheel polarity no unit, 1 or -1
    /// @param right_polarity riht wheel polarity no unit, 1 or -1
    OdometerDifferentialParameters(double left_wheel_diameter, 
    double right_wheel_diameter, double track_width, double left_polarity, double right_polarity) : left_wheel_diameter_(left_wheel_diameter), right_wheel_diameter_(right_wheel_diameter), track_width_(track_width), left_polarity_(left_polarity), right_polarity_(right_polarity) {};

    /// @brief Set the left encoder wheel diameter dimension
    /// @param wheel_diameter Wheel diameter value (mm)
    void set_left_wheel_diameter(double wheel_diameter) { left_wheel_diameter_ = wheel_diameter; }

    /// @brief Return left encoder wheel diameter value in mm.
    /// @return double Wheel diameter value (mm)
    double left_wheel_diameter_mm() const { return left_wheel_diameter_; }

    /// @brief Set the right wheel encoder diameter dimension
    /// @param wheel_diameter Wheel diameter value (mm)
    void set_right_wheel_diameter(double wheel_diameter) { right_wheel_diameter_ = wheel_diameter; }

    /// @brief Return right encoder wheel diameter value in mm.
    /// @return double Wheel diameter value (mm)
    double right_wheel_diameter_mm() const { return right_wheel_diameter_; }

    /// @brief Set the axle track dimension
    /// @param track_width Track width value (mm)
    void set_track_width(double track_width) { track_width_ = track_width; }

    /// @brief Get the axle track value
    /// @return double Track width value (mm)
    double track_width_mm() const { return track_width_; }

    /// @brief Set the left encoder polarity
    /// @param polarity Encoder polarity. -1 or 1.
    void set_left_polarity(double polarity) { left_polarity_ = polarity; }

    /// @brief Get the left encoder polarity
    /// @return double Encoder polarity. -1 or 1
    double left_polarity() const { return left_polarity_; }

    /// @brief Set the right encoder polarity
    /// @param polarity Encoder polarity. -1 or 1.
    void set_right_polarity(double polarity) { right_polarity_ = polarity; }

    /// @brief Get the right encoder polarity
    /// @return double Encoder polarity. -1 or 1
    double right_polarity() const { return right_polarity_; }

private:
    double left_wheel_diameter_;
    double right_wheel_diameter_;
    double track_width_;
    double left_polarity_;
    double right_polarity_;
};

} // namespace odometer

} // namespace cogip
