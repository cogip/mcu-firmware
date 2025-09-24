#pragma once

#include "ztimer.h"

#include "motor_driver.h"

#include "MotorDriverRIOT.hpp"

namespace cogip {

namespace motor {

class MotorDriverDRV8873 : public MotorDriverRIOT
{
  public:
    /// @brief Create a new motor driver object
    /// @param parameters motor driver parameters reference
    explicit MotorDriverDRV8873(const motor_driver_params_t& parameters)
        : MotorDriverRIOT(parameters)
    {
    }

    /// @brief Set motor speed
    /// @param speed speed in % [-100; 100]
    /// @return  0 on success, negative on error
    int set_speed(float speed, int id) override;
};

} // namespace motor

} // namespace cogip

/// @}
