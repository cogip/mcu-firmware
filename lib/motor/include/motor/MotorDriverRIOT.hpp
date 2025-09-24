#pragma once

#include <cstdlib>

#include "motor_driver.h"

#include "MotorDriverInterface.hpp"

namespace cogip {

namespace motor {

class MotorDriverRIOT : public MotorDriverInterface
{
  public:
    /// @brief Create a new motor driver object
    /// @param parameters motor driver parameters reference
    explicit MotorDriverRIOT(const motor_driver_params_t& parameters) : parameters_(parameters) {}

    /// @brief Intialize the motor driver
    /// @return 0 on success, negative on error
    int init() override;

    /// @brief Reset the motor driver
    /// @return 0 on success, negative on error
    int reset() override;

    /// @brief Enable the motor
    /// @param id id of the motor
    /// @return 0 on success, negative on error
    int enable(int id) override;

    /// @brief Disable the motor
    /// @param id id of the motor
    /// @return 0 on success, negative on error
    int disable(int id) override;

    /// @brief Set motor speed
    /// @param speed speed in % [-100; 100]
    /// @param id id of the motor
    /// @return  0 on success, negative on error
    int set_speed(float speed, int id) override;

    /// @brief break the motor
    /// @param id id of the motor
    /// @return 0 on success, negative on error
    int brake(int id) override;

  private:
    motor_driver_params_t parameters_;
    motor_driver_t driver_;
};

} // namespace motor

} // namespace cogip

// @}
