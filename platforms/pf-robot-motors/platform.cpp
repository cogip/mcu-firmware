/* RIOT includes */
#include "log.h"
#include "shell.h"
#include "ztimer.h"

/* Project includes */
#include "app.hpp"
#include "board.h"
#include "platform.hpp"
#include "canpb/ReadBuffer.hpp"
#include "utils.hpp"

/* Platform includes */
#include "pf_actuators.hpp"
#include "pf_positional_actuators.hpp"

#define ENABLE_DEBUG        (0)
#include "debug.h"

// canpb CAN device
static cogip::canpb::CanProtobuf canpb(0);
// canpb default filter
struct can_filter canpb_filter = {0x0, 0x0};

// Thread stacks
static char heartbeat_thread_stack[THREAD_STACKSIZE_DEFAULT];

static bool copilot_connected = false;

char line_buf[SHELL_DEFAULT_BUFSIZE];

// Heartbeat led blinking interval (ms)
constexpr uint16_t heartbeat_leds_interval = 200;
// Heartbeat thread period
constexpr uint16_t heartbeat_period = 1800;

static void *_heartbeat_thread(void *args)
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

void pf_set_copilot_connected(bool connected)
{
    copilot_connected = connected;
}

cogip::canpb::CanProtobuf & pf_get_canpb()
{
    return canpb;
}

/// Start game message handler
static void _handle_game_start([[maybe_unused]] cogip::canpb::ReadBuffer & buffer)
{
    cogip::pf::actuators::enable_all();
}

/// Reset game message handler
static void _handle_game_reset([[maybe_unused]] cogip::canpb::ReadBuffer & buffer)
{
    cogip::pf::actuators::enable_all();
}

/// Start threading sending actuators state.
static void _handle_game_end([[maybe_unused]] cogip::canpb::ReadBuffer & buffer)
{
    cogip::pf::actuators::disable_all();
}

void _handle_copilot_connected(cogip::canpb::ReadBuffer &)
{
    pf_set_copilot_connected(true);
    std::cout << "Copilot connected" << std::endl;
}

void _handle_copilot_disconnected(cogip::canpb::ReadBuffer &)
{
    pf_set_copilot_connected(false);
    std::cout << "Copilot disconnected" << std::endl;
}

void pf_init(void)
{
    thread_create(
        heartbeat_thread_stack,
        sizeof(heartbeat_thread_stack),
        THREAD_PRIORITY_MAIN - 1,
        THREAD_CREATE_STACKTEST,
        _heartbeat_thread,
        NULL,
        "Heartbeat thread"
    );

    /* Initialize CANPB */
    int canpb_res = canpb.init(&canpb_filter);
    if (canpb_res) {
        COGIP_DEBUG_CERR("CAN initialization failed, error: " << canpb_res);
    }
    else {
        canpb.register_message_handler(
            game_reset_uuid,
            cogip::canpb::message_handler_t::create<_handle_game_reset>()
        );
        canpb.register_message_handler(
            game_start_uuid,
            cogip::canpb::message_handler_t::create<_handle_game_start>()
        );
        canpb.register_message_handler(
            game_end_uuid,
            cogip::canpb::message_handler_t::create<_handle_game_end>()
        );
        canpb.register_message_handler(
            copilot_connected_uuid,
            cogip::canpb::message_handler_t::create<_handle_copilot_connected>()
            );
        canpb.register_message_handler(
            copilot_disconnected_uuid,
            cogip::canpb::message_handler_t::create<_handle_copilot_disconnected>()
            );

        canpb.start_reader();
    }

    cogip::pf::actuators::init();
}

void pf_init_tasks(void)
{
}
