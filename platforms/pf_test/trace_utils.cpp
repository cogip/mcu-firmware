#include "trace_utils.hpp"

// RIOT includes
#include "riot/thread.hpp"

// Application includes
#include "tracefd/tracefd.hpp"
#include "platform.hpp"
#include "obstacles/obstacles.hpp"

/* Trace filename */
#define TRACE_FILE "trace.txt"

/* Trace file descriptor on sdcard */
static cogip::tracefd::File *tracefd_sdcard;

/* Periodic task */
#define TASK_PERIOD_MS 60

/* Thread stack */
static char trace_thread_stack[THREAD_STACKSIZE_LARGE];

/* Thread priority */
#define TRACE_PRIO (THREAD_PRIORITY_MAIN - 1)

/* Thread loop */
static void *_thread_trace(void *arg)
{
    (void)arg;

    while (true) {
        if (pf_trace_on()) {
            pf_print_state(cogip::tracefd::out);
        }
        if (tracefd_sdcard) {
            pf_print_state(*tracefd_sdcard);
        }
        riot::this_thread::sleep_for(std::chrono::milliseconds(TASK_PERIOD_MS));
    }

    return NULL;
}

void trace_start(void)
{
    try {
        tracefd_sdcard = new cogip::tracefd::File(TRACE_FILE);
        tracefd_sdcard->open();
    }
    catch(std::runtime_error &) {
        tracefd_sdcard = nullptr;
    }

    /* Start the trace thread */
    thread_create(
        trace_thread_stack,
        sizeof(trace_thread_stack),
        TRACE_PRIO,
        0,
        _thread_trace,
        NULL,
        "Trace thread"
        );
}
