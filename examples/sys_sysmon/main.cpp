// System includes
#include <cstdio>
#include <iostream>

// RIOT includes
#include <periph/can.h>
#include <shell.h>

// Project includes
#include "sysmon/sysmon.hpp"

#ifdef MODULE_CANPB
#include "canpb/CanProtobuf.hpp"

cogip::canpb::CanProtobuf canpb(0);
// canpb default filter
struct can_filter canpb_filter = {0x0, 0x0};

#endif

static int cmd_display_heap_status(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    cogip::sysmon::display_heap_status();

    return 0;
}

static int cmd_display_threads_status(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    cogip::sysmon::display_threads_status();

    return 0;
}

static const shell_command_t shell_commands[] = {
    { "heap_status", "Display heap memory status", cmd_display_heap_status },
    { "threads_status", "Display threads status", cmd_display_threads_status },
    { NULL, NULL, NULL }
};

int main(void)
{
    puts("\n== System monitoring example ==");

#ifdef MODULE_CANPB
    bool res = canpb.init(&canpb_filter);
    if (res) {
        std::cerr << "CAN initialization status: " << res << std::endl;
        exit(1);
    }

    cogip::sysmon::register_canpb(&canpb);
#endif

    // Start the monitoring thread
    cogip::sysmon::sysmon_start();

    /* Start shell */
    char line_buf[SHELL_DEFAULT_BUFSIZE];
    shell_run(shell_commands, line_buf, SHELL_DEFAULT_BUFSIZE);

    return 0;
}
