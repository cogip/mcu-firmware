// Copyright (C) 2024 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

#include "pf_actuators.hpp"
#include "pf_positional_actuators.hpp"

#include "actuator/Lift.hpp"
#include "actuator/LiftsLimitSwitchesManager.hpp"
#include "actuator/PositionalActuator.hpp"

#include "actuators_motors_params.hpp"
#include "platform.hpp"
#include "motion_control_common/BaseController.hpp"
#include "motion_control_common/BaseControllerEngine.hpp"

#include "etl/map.h"
#include "etl/pool.h"
#include "log.h"

// RIOT includes
#include <event.h>
#include <ztimer.h>
#include <motor_driver.h>
#include <periph/qdec.h>

namespace cogip {
namespace pf {
namespace actuators {
namespace positional_actuators {

extern "C" {
extern int32_t qdecs_value[QDEC_NUMOF];
}

// Actuator state protobuf message
static PB_ActuatorState _pb_actuator_state;

/// Lifts memory pool
static etl::pool<cogip::actuators::positional_actuators::Lift, CONFIG_ACTUATOR_LIFT_NUMBER> _lifts_pool;

/// Define total number of actuators
static constexpr uint8_t _actuator_total_number = CONFIG_ACTUATOR_LIFT_NUMBER;
/// Positional actuators map
static etl::map<cogip::actuators::Enum, cogip::actuators::positional_actuators::PositionalActuator *, _actuator_total_number> _positional_actuators;

void disable_all() {
    for (auto & iterator: _positional_actuators) {
        cogip::actuators::positional_actuators::PositionalActuator *positional_actuator = iterator.second;
        positional_actuator->disable();
    }
}

void enable_all() {
    for (auto & iterator: _positional_actuators) {
        cogip::actuators::positional_actuators::PositionalActuator *positional_actuator = iterator.second;
        positional_actuator->enable();
    }
}

void init_sequence(void) {
    for (auto & iterator: _positional_actuators) {
        cogip::actuators::positional_actuators::PositionalActuator *positional_actuator = iterator.second;
        positional_actuator->init();
    }
}

int create_lift(
    cogip::actuators::Enum id,
    const cogip::actuators::positional_actuators::LiftParameters& lift_params) {
    // Create Lift
    _positional_actuators[id] = _lifts_pool.create(
        lift_params
    );

    if (!_positional_actuators[id]) {
        LOG_ERROR("Error creating lift");
        return -ENOMEM;
    }

    _positional_actuators[id]->enable();

    return 0;
}

void init(void) {
    // Init lift GPIO handler
    cogip::actuators::positional_actuators::LiftsLimitSwitchesManager::instance().init();

    // Enable all actuators
    enable_all();
}

bool contains(cogip::actuators::Enum id) {
    return _positional_actuators.contains(id);
}

cogip::actuators::positional_actuators::PositionalActuator & get(cogip::actuators::Enum id) {
    return *_positional_actuators[id];
}

void send_state(cogip::actuators::Enum positional_actuator) {
    // Protobuf CAN interface
    static cogip::canpb::CanProtobuf & canpb = pf_get_canpb();

    // Send protobuf message
    _pb_actuator_state.clear();
    positional_actuators::get(positional_actuator).pb_copy(_pb_actuator_state.mutable_positional_actuator());
    if (!canpb.send_message(actuator_state_uuid, &_pb_actuator_state)) {
        LOG_ERROR("actuator_state_uuid message not sent");
    }
}

} // namespace positional_actuators
} // namespace actuators
} // namespace pf
} // namespace cogip
