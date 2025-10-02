// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup     platforms_pf-common
/// @{
/// @file
/// @brief       Common platform base functionality implementations
/// @author      Mathis LECRIVAIN <lecrivain.mathis@gmail.com>

// RIOT includes
#include "board.h"
#include "log.h"
#include "thread.h"
#include "ztimer.h"

// Project includes
#include "pf_common/platform_common.hpp"
#include "pf_common/uuids.hpp"

namespace cogip {
namespace pf_common {

// Static variables
static bool copilot_connected = false;
static copilot_callback_t on_copilot_connected_callback;
static copilot_callback_t on_copilot_disconnected_callback;

// canpb CAN device
static canpb::CanProtobuf canpb(0);
// canpb default filter
static struct can_filter canpb_filter = {0x0, 0x0};

// Thread stack
static char heartbeat_thread_stack[THREAD_STACKSIZE_DEFAULT];

// Heartbeat configuration
constexpr uint16_t heartbeat_leds_interval = 200;
constexpr uint16_t heartbeat_period = 1800;

/// @brief Set/unset copilot connected status (private)
/// @param[in] connected  copilot connected if true, not connected otherwise
static void set_copilot_connected(bool connected)
{
    copilot_connected = connected;
}

/// @brief Default handler for copilot connected message (private)
/// @param[in] buffer ReadBuffer containing the message
static void handle_copilot_connected(canpb::ReadBuffer& buffer)
{
    set_copilot_connected(true);
    LOG_INFO("Copilot connected\n");

    // Call custom callback if registered
    if (on_copilot_connected_callback) {
        on_copilot_connected_callback(buffer);
    }
}

/// @brief Default handler for copilot disconnected message (private)
/// @param[in] buffer ReadBuffer containing the message
static void handle_copilot_disconnected(canpb::ReadBuffer& buffer)
{
    set_copilot_connected(false);
    LOG_INFO("Copilot disconnected\n");

    // Call custom callback if registered
    if (on_copilot_disconnected_callback) {
        on_copilot_disconnected_callback(buffer);
    }
}

/// @brief Heartbeat thread function
/// @param[in] args Unused
/// @return nullptr
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

    return nullptr;
}

bool is_copilot_connected()
{
    return copilot_connected;
}

canpb::CanProtobuf& get_canpb()
{
    return canpb;
}

int pf_init(copilot_callback_t on_connect, copilot_callback_t on_disconnect)
{
    // Store custom callbacks
    on_copilot_connected_callback = on_connect;
    on_copilot_disconnected_callback = on_disconnect;

    // Initialize CAN with default filter
    int canpb_res = canpb.init(&canpb_filter);
    if (canpb_res) {
        LOG_ERROR("CAN initialization failed, error: %d\n", canpb_res);
        return canpb_res;
    }

    // Register common message handlers
    canpb.register_message_handler(copilot_connected_uuid,
                                   canpb::message_handler_t::create<handle_copilot_connected>());
    canpb.register_message_handler(copilot_disconnected_uuid,
                                   canpb::message_handler_t::create<handle_copilot_disconnected>());

    return 0;
}

void pf_init_tasks()
{
    // Start heartbeat thread
    thread_create(heartbeat_thread_stack, sizeof(heartbeat_thread_stack), THREAD_PRIORITY_MAIN - 1,
                  THREAD_CREATE_STACKTEST, _heartbeat_thread, nullptr, "Heartbeat thread");

    // Start CAN reader
    canpb.start_reader();
}

} // namespace pf_common
} // namespace cogip

cogip::canpb::CanProtobuf& pf_get_canpb()
{
    return cogip::pf_common::get_canpb();
}

/// @}
