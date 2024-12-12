#pragma once

#include "motor/MotorInterface.hpp"
#include "motor/MotorDriverDRV8873.hpp"

namespace cogip {

namespace motor {

class MotorRIOT: public MotorInterface {
public:
    MotorRIOT(MotorDriverRIOT& driver, int id): driver_(driver), id_(id)
    {
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
    int speed(int speed)
    {
        return driver_.speed(speed, id_);
    }
    
    /// @brief break the motor
    /// @return 0 on success, negative on error
    int brake()
    {
        return driver_.brake(id_);
    }

private:
    MotorDriverRIOT &driver_;
    int id_;
};

} // namespace motor

} // namespace cogip

// @}