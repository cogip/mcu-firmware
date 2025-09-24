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
#include "shell.h"
#include "ztimer.h"

/* Project includes */
#include "app.hpp"
#include "board.h"
#include "canpb/ReadBuffer.hpp"
#include "platform.hpp"
#include "utils.hpp"

#define ENABLE_DEBUG (0)
#include "debug.h"

#include "pf_power_supply.hpp"

// canpb CAN device
static cogip::canpb::CanProtobuf canpb(0);
// canpb default filter
struct can_filter canpb_filter = {0x0, 0x0};

// Thread stacks
static char heartbeat_thread_stack[THREAD_STACKSIZE_DEFAULT];

// Heartbeat led blinking interval (ms)
constexpr uint16_t heartbeat_leds_interval = 200;
// Heartbeat thread period
constexpr uint16_t heartbeat_period = 1800;

static void* _heartbeat_thread(void* args)
{
    (void)args;

    // Init loop iteration start time
    ztimer_now_t loop_start_time = ztimer_now(ZTIMER_MSEC);

    while (true) {
        gpio_set(HEARTBEAT_LED);
        ztimer_sleep(ZTIMER_MSEC, heartbeat_leds_interval);
        gpio_clear(HEARTBEAT_LED);
        ztimer_sleep(ZTIMER_MSEC, heartbeat_leds_interval);
        gpio_set(HEARTBEAT_LED);
        ztimer_sleep(ZTIMER_MSEC, heartbeat_leds_interval);
        gpio_clear(HEARTBEAT_LED);

        // Wait thread period to end
        ztimer_periodic_wakeup(ZTIMER_MSEC, &loop_start_time, heartbeat_period);
    }

    return 0;
}

cogip::canpb::CanProtobuf& pf_get_canpb()
{
    return canpb;
}

void _handle_copilot_connected(cogip::canpb::ReadBuffer&)
{
    cogip::pf::power_supply::send_emergency_stop_status();
    cogip::pf::power_supply::send_power_rails_status();
    cogip::pf::power_supply::send_power_source_status();
    LOG_INFO("Copilot connected");
}

void _handle_copilot_disconnected(cogip::canpb::ReadBuffer&)
{
    LOG_INFO("Copilot disconnected");
}

void pf_init(void)
{
    thread_create(heartbeat_thread_stack, sizeof(heartbeat_thread_stack), THREAD_PRIORITY_MAIN - 1,
                  THREAD_CREATE_STACKTEST, _heartbeat_thread, NULL, "Heartbeat thread");

    /* Initialize CANPB */
    int canpb_res = canpb.init(&canpb_filter);
    if (canpb_res) {
        LOG_ERROR("CAN initialization failed, error: %d\n", canpb_res);
    } else {
        canpb.register_message_handler(
            copilot_connected_uuid,
            cogip::canpb::message_handler_t::create<_handle_copilot_connected>());
        canpb.register_message_handler(
            copilot_disconnected_uuid,
            cogip::canpb::message_handler_t::create<_handle_copilot_disconnected>());

        canpb.start_reader();
    }

    cogip::pf::power_supply::pf_init_power_supply();
}

void pf_init_tasks(void)
{
    cogip::pf::power_supply::pf_init_power_supply_tasks();
}
