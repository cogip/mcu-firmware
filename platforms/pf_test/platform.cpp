/* RIOT includes */
#include "log.h"

/* Project includes */
#include "app.hpp"
#include "motion_control.hpp"
#include "platform.hpp"
#include "shell_menu/shell_menu.hpp"
#include "uartpb/ReadBuffer.hpp"
#include "utils.hpp"

/* Platform includes */
#include "trace_utils.hpp"

#include "PB_Command.hpp"

#ifdef MODULE_SHELL_PLATFORMS
#include "shell_platforms.hpp"
#endif /* MODULE_SHELL_PLATFORMS */

#define ENABLE_DEBUG        (0)
#include "debug.h"

// uartpb UART device
static cogip::uartpb::UartProtobuf uartpb(UART_DEV(1));

/* Thread stacks */
char countdown_thread_stack[THREAD_STACKSIZE_DEFAULT];

static bool copilot_connected = false;

bool pf_trace_on(void)
{
    return copilot_connected;
}

void pf_set_copilot_connected(bool connected)
{
    copilot_connected = connected;
}

cogip::uartpb::UartProtobuf & pf_get_uartpb()
{
    return uartpb;
}

void handle_copilot_connected(cogip::uartpb::ReadBuffer &)
{
    pf_set_copilot_connected(true);
    std::cout << "Copilot connected" << std::endl;
    if (cogip::shell::current_menu) {
        cogip::shell::current_menu->send_pb_message();
    }
}

void handle_copilot_disconnected(cogip::uartpb::ReadBuffer &)
{
    pf_set_copilot_connected(false);
    std::cout << "Copilot disconnected" << std::endl;
}

void pf_init(void)
{
    /* Initialize UARTPB */
    bool uartpb_res = uartpb.connect();
    if (! uartpb_res) {
        COGIP_DEBUG_CERR("UART initialization failed, error: " << uartpb_res);
    }
    else {
        cogip::shell::register_uartpb(&uartpb);
        uartpb.register_message_handler(
            copilot_connected_uuid,
            cogip::uartpb::message_handler_t::create<handle_copilot_connected>()
            );
        uartpb.register_message_handler(
            copilot_disconnected_uuid,
            cogip::uartpb::message_handler_t::create<handle_copilot_disconnected>()
            );
        uartpb.register_message_handler(
            cogip::pf::motion_control::pose_uuid,
            cogip::uartpb::message_handler_t::create<cogip::pf::motion_control::pf_handle_target_pose>()
            );
        uartpb.register_message_handler(
            cogip::pf::motion_control::start_pose_uuid,
            cogip::uartpb::message_handler_t::create<cogip::pf::motion_control::pf_handle_start_pose>()
            );

        uartpb.start_reader();
        uartpb.send_message(reset_uuid);
    }

#ifdef MODULE_SHELL_PLATFORMS
    pf_shell_init();
#endif /* MODULE_SHELL_PLATFORMS */

    cogip::pf::motion_control::pf_init_motion_control();
}

void pf_init_tasks(void)
{
    trace_start();

    cogip::pf::motion_control::pf_start_motion_control();
}
