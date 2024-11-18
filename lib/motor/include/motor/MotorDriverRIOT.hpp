#pragma once

#include <cstdlib>

#include "motor_driver.h"

#include "MotorDriverInterface.hpp"

namespace cogip {

namespace motor {

class MotorDriverRIOT: public MotorDriverInterface {
public:
    /// @brief Create a new motor driver object
    /// @param parameters motor driver parameters reference
    explicit MotorDriverRIOT(const motor_driver_params_t& parameters): parameters_(parameters)
    {
    }
    
    /// @brief Intialize the motor driver
    /// @return 0 on success, negative on error
    int init() override
    {
        return motor_driver_init(&driver_, &parameters_);
    }
    
    /// @brief Reset the motor driver
    /// @return 0 on success, negative on error
    int reset() override
    {
        /// There is no reset function for RIOT motor driver
        return 0;
    }
    
    /// @brief Enable the motor
    /// @param id id of the motor
    /// @return 0 on success, negative on error
    int enable(int id) override
    {
        motor_enable(&driver_, id);
        return 0;
    }

    /// @brief Disable the motor
    /// @param id id of the motor
    /// @return 0 on success, negative on error
    int disable(int id) override
    {
        motor_disable(&driver_, id);
        return 0;
    }

    /// @brief Set motor speed
    /// @param speed speed in % [-100; 100]
    /// @param id id of the motor
    /// @return  0 on success, negative on error
    int speed(int speed, int id) override
    {
        // Convert speed in percent to pwm value
        int32_t pwm_value = (speed * (int32_t)parameters_.pwm_resolution) / 100;
        
        // limit pwm value in order to ensure range between [-parameters_.pwm_resolution; parameters_.pwm_resolution]
        if ((uint32_t)abs(pwm_value) > parameters_.pwm_resolution)
        {
            pwm_value = (speed < 0 ? -1 : 1) * parameters_.pwm_resolution;
        }

        return motor_set(&driver_, id, pwm_value);
    }
    
    /// @brief break the motor
    /// @param id id of the motor
    /// @return 0 on success, negative on error
    int brake(int id) override
    {
        return motor_brake(&driver_, id);
    }
private:
    motor_driver_params_t parameters_;
    motor_driver_t driver_;
};

} // namespace motor

} // namespace cogip

// @}
