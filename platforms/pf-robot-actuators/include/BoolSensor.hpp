// Copyright (C) 2024 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup     platforms_pf-robot-sensors
/// @{
/// @file
/// @brief       Boolean sensor class definition inheriting from base Sensor class.
/// @author      Gilles DOFFE <g.doffe@gmail.com>

#pragma once

#include "Sensor.hpp"
#include "PB_Actuators.hpp"

namespace cogip {
namespace pf {
namespace sensors {
namespace bool_sensors {

class BoolSensor : public Sensor {
public:
    /// Constructor
    explicit BoolSensor(
        Enum id,                                ///< [in] sensor id
        bool initial_state = false,             ///< [in] initial state of the sensor
        send_state_cb_t send_state_cb = nullptr ///< [in] send state callback
    ) : Sensor(id, send_state_cb), state_(initial_state) {}

    /// Get sensor state
    bool state() const { return state_; }

    /// Set sensor state
    void set_state(bool new_state) { state_ = new_state; }

    /// Copy state to Protobuf message.
    void pb_copy(
        PB_BoolSensor & pb_bool_sensor  ///< [out] Protobuf message to fill
    ) const;

private:
    bool state_;    ///< Current state of the boolean sensor
};

} // namespace bool_sensors
} // namespace sensors
} // namespace pf
} // namespace cogip

/// @}
