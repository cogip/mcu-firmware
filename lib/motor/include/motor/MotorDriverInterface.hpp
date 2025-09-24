#pragma once

namespace cogip {

namespace motor {

class MotorDriverInterface
{
  public:
    /// @brief Virtual destructor
    ~MotorDriverInterface() {}

    /// @brief Intialize the motor driver
    /// @return 0 on success, negative on error
    virtual int init() = 0;

    /// @brief Reset the motor driver
    /// @return 0 on success, negative on error
    virtual int reset() = 0;

    /// @brief Enable the motor
    /// @param id id of the motor
    /// @return 0 on success, negative on error
    virtual int enable(int id) = 0;

    /// @brief Disable the motor
    /// @param id id of the motor
    /// @return 0 on success, negative on error
    virtual int disable(int id) = 0;

    /// @brief Set motor speed
    /// @param speed speed in % [-100; 100]
    /// @param id id of the motor
    /// @return  0 on success, negative on error
    virtual int set_speed(float speed, int id) = 0;

    /// @brief break the motor
    /// @param id id of the motor
    /// @return 0 on success, negative on error
    virtual int brake(int id) = 0;
};

} // namespace motor

} // namespace cogip

// @}
