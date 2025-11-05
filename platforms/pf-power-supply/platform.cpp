///  Copyright (C) 2025 COGIP Robotics association
///
///  This file is subject to the terms and conditions of the GNU Lesser
///  General Public License v2.1. See the file LICENSE in the top level
///  directory for more details.

///
///  @file
///  @ingroup     platforms_pf-power-supply
///  @brief       Power supply control platform implementation
///
///  @author      Mathis LECRIVAIN <lecrivain.mathis@gmail.com>

/* RIOT includes */
#include "log.h"

/* Project includes */
#include "canpb/ReadBuffer.hpp"
#include "pf_common/platform_common.hpp"
#include "platform.hpp"

/* Platform includes */
#include "pf_power_supply.hpp"

/// Custom copilot connected callback for power supply
static void _on_copilot_connected(cogip::canpb::ReadBuffer&)
{
    cogip::pf::power_supply::send_emergency_stop_status();
    cogip::pf::power_supply::send_power_rails_status();
    cogip::pf::power_supply::send_power_source_status();
}

void pf_init(void)
{
    /* Initialize common platform (CAN, heartbeat, copilot handlers) with custom callback */
    int ret = cogip::pf_common::pf_init(
        cogip::pf_common::copilot_callback_t::create<_on_copilot_connected>());
    if (ret) {
        LOG_ERROR("Common platform initialization failed, error: %d\n", ret);
    }

    cogip::pf::power_supply::pf_init_power_supply();
}

void pf_init_tasks(void)
{
    cogip::pf_common::pf_init_tasks();

    cogip::pf::power_supply::pf_init_power_supply_tasks();
}
