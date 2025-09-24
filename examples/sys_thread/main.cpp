// System includes
#include "log.h"
#include <cstdio>
#include <inttypes.h>

// RIOT includes
#include <periph/can.h>
#include <shell.h>

// Project includes
#include "sysmon/sysmon.hpp"
#include "thread/thread.hpp"
#ifdef MODULE_CANPB
#include "canpb/CanProtobuf.hpp"
#endif

// Periodic task
#define TASK_PERIOD_SEC (1)

// Thread stack
static char _thread_buggy_stack[THREAD_STACKSIZE_SMALL];

cogip::canpb::CanProtobuf canpb(0);
// canpb default filter
struct can_filter canpb_filter = {0x0, 0x0};

static int cmd_display_heap_status(int argc, char** argv)
{
    (void)argc;
    (void)argv;

    cogip::sysmon::display_heap_status();

    return 0;
}

static int cmd_display_threads_status(int argc, char** argv)
{
    (void)argc;
    (void)argv;

    cogip::sysmon::display_threads_status();

    return 0;
}

static const shell_command_t shell_commands[] = {
    {"heap_status", "Display heap memory status", cmd_display_heap_status},
    {"threads_status", "Display threads status", cmd_display_threads_status},
    {NULL, NULL, NULL}};

static void* _thread_buggy(void* data)
{
    (void)data;

    ztimer_now_t loop_start_time = ztimer_now(ZTIMER_SEC);

    while (true) {
        // Sleep 3 seconds in a thread with a period of 1 second.
        // Should display latency overshot warning
        ztimer_sleep(ZTIMER_SEC, 3);

        cogip::thread::thread_ztimer_periodic_wakeup(ZTIMER_SEC, &loop_start_time, 1);
    }

    return 0;
}

static void thread_buggy_start(void)
{
    thread_create(_thread_buggy_stack, sizeof(_thread_buggy_stack), THREAD_PRIORITY_MAIN - 1,
                  THREAD_CREATE_STACKTEST, _thread_buggy, NULL, "Buggy thread");
}

int main(void)
{
    LOG_INFO("== System thread monitoring example ==\n");

    bool res = canpb.init(&canpb_filter);
    if (!res) {
        LOG_ERROR("CAN initialization status: %d\n", res);
        exit(1);
    }

#ifdef MODULE_CANPB
    cogip::sysmon::register_canpb(&canpb);
#endif

    // Start the monitoring thread
    cogip::sysmon::sysmon_start();

    thread_buggy_start();

    /* Start shell */
    char line_buf[SHELL_DEFAULT_BUFSIZE];
    shell_run(shell_commands, line_buf, SHELL_DEFAULT_BUFSIZE);

    return 0;
}
