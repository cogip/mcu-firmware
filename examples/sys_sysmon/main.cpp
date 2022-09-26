// System includes
#include <cstdio>
#include <iostream>

// RIOT includes
#include <periph/uart.h>
#include <shell.h>

// Project includes
#include "sysmon/sysmon.hpp"
#ifdef MODULE_UARTPB
#include "uartpb/UartProtobuf.hpp"
#endif

cogip::uartpb::UartProtobuf uartpb(UART_DEV(1));

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

    bool res = uartpb.connect();
    if (! res) {
        std::cerr << "UART initialization status: " << res << std::endl;
        exit(1);
    }

#ifdef MODULE_UARTPB
    cogip::sysmon::register_uartpb(&uartpb);
#endif

    // Start the monitoring thread
    cogip::sysmon::sysmon_start();

    /* Start shell */
    char line_buf[SHELL_DEFAULT_BUFSIZE];
    shell_run(shell_commands, line_buf, SHELL_DEFAULT_BUFSIZE);

    return 0;
}
