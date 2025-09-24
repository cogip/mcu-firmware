#pragma once

#include "motor/MotorDriverDRV8873.hpp"
#include "motor/MotorInterface.hpp"

namespace cogip {

namespace motor {

class MotorRIOT : public MotorInterface
{
  public:
    MotorRIOT(MotorDriverRIOT& driver, int id) : driver_(driver), id_(id) {}

    /// @brief Init the motor and its driver
    /// @return 0 on success, negative on error
    int init()
    {
        return driver_.init();
    }

    /// @brief Enable the motor
    /// @return 0 on success, negative on error
    int enable()
    {
        return driver_.enable(id_);
    }

    /// @brief Enable the motor
    /// @return 0 on success, negative on error
    int disable()
    {
        return driver_.disable(id_);
    }

    /// @brief Set motor speed
    /// @param speed speed in % [-100; 100]
    /// @return  0 on success, negative on error
    int set_speed(float speed)
    {
        return driver_.set_speed(speed, id_);
    }

    /// @brief break the motor
    /// @return 0 on success, negative on error
    int brake()
    {
        return driver_.brake(id_);
    }

  private:
    MotorDriverRIOT& driver_;
    int id_;
};

} // namespace motor

} // namespace cogip

// @}
