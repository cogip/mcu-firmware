// Copyright (C) 2024 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

#include "Pump.hpp"
#include "vacuum_pump_params.h"

#include <iostream>

namespace cogip {
namespace pf {
namespace actuators {
namespace pumps {

std::ostream& operator << (std::ostream& os, Enum id) {
    os << static_cast<std::underlying_type_t<Enum>>(id);
    return os;
}

Pump::Pump(Enum id, GroupEnum group, uint8_t order) : Actuator(group, order), id_(id) {
    auto pump_id = static_cast<std::underlying_type_t<Enum>>(id);
    vacuum_pump_init(pump_id, &vacuum_pump_params[pump_id]);
    deactivate();
};

void Pump::activate(bool enable) {
    activated_ = enable;
    if (enable) {
        vacuum_pump_start(static_cast<vacuum_pump_t>(id_));
    }
    else {
        vacuum_pump_stop(static_cast<vacuum_pump_t>(id_));
    }
}

void Pump::deactivate() {
    activate(false);
}

bool Pump::under_pressure() const {
    return vacuum_pump_is_under_pressure(static_cast<vacuum_pump_t>(id_));
}

void Pump::pb_copy(PB_Pump & pb_pump) const {
    pb_pump.set_group(static_cast<PB_ActuatorsGroupEnum>(group_));
    pb_pump.set_order(order_);
    pb_pump.set_id(static_cast<PB_PumpEnum>(id_));
    pb_pump.set_activated(activated_);
    pb_pump.set_under_pressure(under_pressure());
}

} // namespace pumps
} // namespace actuators
} // namespace pf
} // namespace cogip
