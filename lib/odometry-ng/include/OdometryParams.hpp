#pragma once

namespace cogip {

namespace odometry {

class OdometryParams {
public:
    /**
     * @brief Construct a new Odometry Params object
     *
     * @param wheel_diameter Wheel diameter value (mm)
     * @param axle_track Axle track value (mm)
     */
    OdometryParams(double wheel_diameter, double axle_track) : wheel_diameter_(wheel_diameter), axle_track_(axle_track) {};
    
    /**
     * @brief Set the wheel diameter dimension
     *
     * @param wheel_diameter Wheel diameter value (mm)
     */
    void set_wheel_diameter(double wheel_diameter) { wheel_diameter_ = wheel_diameter; }
    
    /**
     * @brief Return encoder wheel diameter value in mm.
     *
     * @return double Wheel diameter value (mm)
     */
    double wheel_diameter_mm() const { return wheel_diameter_; }


    /**
     * @brief Set the axle track dimension
     *
     * @param axle_track Axle track value (mm)
     */
    void set_axle_track(double axle_track) { axle_track_ = axle_track; }

    /**
     * @brief Get the axle track value
     *
     * @return double Axle track value (mm)
     */
    double axle_track_mm() const { return axle_track_; }

private:
    double wheel_diameter_;
    double axle_track_;
};

} // namespace odometry

} // namespace cogip
