// Copyright (C) 2023 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup    speed_filter Speed filter parameters
/// @{
/// @file
/// @brief      Speed and acceleration limits
/// @author     Gilles DOFFE <g.doffe@gmail.com>

#pragma once

namespace cogip {

namespace motion_control {

/// Speed and acceleration limits
class SpeedFilterParameters
{
  public:
    /// Constructor
    explicit SpeedFilterParameters(float min_speed = 0.0,        ///< [in]  see max_speed_
                                   float max_speed = 0.0,        ///< [in]  see max_speed_
                                   float max_acceleration = 0.0, ///< [in]  see max_acceleration_
                                   bool anti_blocking = false,
                                   ///< [in]   anti-blocking flag
                                   float anti_blocking_speed_threshold = 0,
                                   ///< [in]   new anti blocking speed threshold
                                   float anti_blocking_error_threshold = 0,
                                   ///< [in]   new anti blocking error threshold
                                   uint32_t anti_blocking_blocked_cycles_nb_threshold = 0
                                   ///< [in]   new anti blocking blocked cycles threshold
                                   )
        : min_speed_(min_speed), max_speed_(max_speed), max_acceleration_(max_acceleration),
          anti_blocking_(anti_blocking),
          anti_blocking_speed_threshold_(anti_blocking_speed_threshold),
          anti_blocking_error_threshold_(anti_blocking_error_threshold),
          anti_blocking_blocked_cycles_nb_threshold_(anti_blocking_blocked_cycles_nb_threshold){};

    /// Get minimum speed
    /// return minimum speed
    float min_speed() const
    {
        return min_speed_;
    };

    /// Get maximum speed
    /// return maximum speed
    float max_speed() const
    {
        return max_speed_;
    };

    /// Get maximum acceleration
    /// return maximum acceleration
    float max_acceleration() const
    {
        return max_acceleration_;
    };

    /// Set minimum speed
    void set_min_speed(float min_speed ///< [in]   minimum speed
    )
    {
        min_speed_ = min_speed;
    };

    /// Set maximum speed
    void set_max_speed(float max_speed ///< [in]   maximum speed
    )
    {
        max_speed_ = max_speed;
    };

    /// Set maximum acceleration
    void set_max_acceleration(float max_acceleration ///< [in]   maximum acceleration
    )
    {
        max_acceleration_ = max_acceleration;
    };

    /// Get anti blocking activation flag
    /// return true if activated, false otherwise
    bool anti_blocking() const
    {
        return anti_blocking_;
    };

    /// Set anti blocking activation flag
    void set_anti_blocking(bool anti_blocking ///< [in]   anti blocking on/off
    )
    {
        anti_blocking_ = anti_blocking;
    };

    /// Get anti blocking speed threshold
    /// return anti blocking speed threshold
    float anti_blocking_speed_threshold() const
    {
        return anti_blocking_speed_threshold_;
    };

    /// Set anti blocking speed threshold
    void set_anti_blocking_speed_threshold(
        float anti_blocking_speed_threshold ///< [in]   new anti blocking speed
                                            ///< threshold
    )
    {
        anti_blocking_speed_threshold_ = anti_blocking_speed_threshold;
    };

    /// Get anti blocking error threshold
    /// return anti blocking error threshold
    float anti_blocking_error_threshold() const
    {
        return anti_blocking_error_threshold_;
    };

    /// Set anti blocking error threshold
    void set_anti_blocking_error_threshold(
        float anti_blocking_error_threshold ///< [in]   new anti blocking error
                                            ///< threshold
    )
    {
        anti_blocking_error_threshold_ = anti_blocking_error_threshold;
    };

    /// Get anti blocking blocked cycles number threshold
    /// return anti blocking blocked cycles number threshold
    float anti_blocking_blocked_cycles_nb_threshold() const
    {
        return anti_blocking_blocked_cycles_nb_threshold_;
    };

    /// Set anti blocking blocked cycles number threshold
    void set_anti_blocking_blocked_cycles_nb_threshold(
        float anti_blocking_blocked_cycles_nb_threshold ///< [in]   new anti
                                                        ///< blocking blocked cycles
                                                        ///< number threshold
    )
    {
        anti_blocking_blocked_cycles_nb_threshold_ = anti_blocking_blocked_cycles_nb_threshold;
    };

  private:
    /// minimum speed the robot should reach
    float min_speed_;

    /// maximum speed the robot can reach
    float max_speed_;

    /// maximum robot acceleration allowed
    float max_acceleration_;

    /// Anti blocking on ?
    bool anti_blocking_;

    /// anti blocking speed threshold
    float anti_blocking_speed_threshold_;

    /// anti blocking error threshold
    float anti_blocking_error_threshold_;

    /// anti blocking blocked cycles number threshold
    uint32_t anti_blocking_blocked_cycles_nb_threshold_;
};

} // namespace motion_control

} // namespace cogip

/// @}
