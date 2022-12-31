// Copyright (C) 2023 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup     platforms_pegasus
/// @{
/// @file
/// @brief       C++ class representing a pump using vacuum_pump driver.
/// @author      Eric Courtois <eric.courtois@gmail.com>

#pragma once

#include "vacuum_pump.h"

#include "Actuator.hpp"
#include "PB_Actuators.hpp"

#include <iosfwd>

namespace cogip {
namespace pf {
namespace actuators {
namespace pumps {

enum class Enum: vacuum_pump_t;

std::ostream& operator << (std::ostream& os, Enum id);

/// Class representing a pump using vacuum_pump driver.
class Pump: public Actuator {
public:
    /// Constructor.
    Pump(
        Enum id,           ///< [in] pump id
        GroupEnum group,   ///< [in] actuator group
        uint8_t order = 0  ///< [in] order in actuator group
    );

    /// Activate the pump.
    void activate(
        bool enable = true  ///< activate pump if true, deactivate otherwise
    );

    /// Deactivate the pump.
    void deactivate();

    /// Pump is under pressure or not.
    bool under_pressure() const;

    /// Copy data to Protobuf message.
    void pb_copy(
        PB_Pump & pb_pump  ///< [out] Protobuf message to fill
    ) const;

private:
    Enum id_;         ///< pump id
    bool activated_;  ///< whether pump is activated or not
};

} // namespace pumps
} // namespace actuators
} // namespace app
} // namespace cogip

/// @}
