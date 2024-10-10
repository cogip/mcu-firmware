#pragma once

namespace cogip {

namespace odometry {

class OdometryParams {
public:
    /**
     * @brief Construct a new Odometry Params object
     *
     * @param axle_track Axle track value (mm)
     */
    OdometryParams(double axle_track) : axle_track_(axle_track) {};

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
    double get_axle_track() const { return axle_track_; }

private:
    double axle_track_;
};

} // namespace odometry

} // namespace cogip
