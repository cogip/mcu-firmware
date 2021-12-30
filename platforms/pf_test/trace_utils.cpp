#include "trace_utils.hpp"

// RIOT includes
#include "riot/thread.hpp"

// Application includes
#include "platform.hpp"

/* Periodic task */
#define TASK_PERIOD_MS 60

/* Thread stack */
static char trace_thread_stack[THREAD_STACKSIZE_MEDIUM];

/* Thread priority */
#define TRACE_PRIO (THREAD_PRIORITY_MAIN - 1)

/* Thread loop */
static void *_thread_trace(void *arg)
{
    (void)arg;

    while (true) {
        if (pf_trace_on()) {
            pf_send_pb_state();
        }
        riot::this_thread::sleep_for(std::chrono::milliseconds(TASK_PERIOD_MS));
    }

    return NULL;
}

void trace_start(void)
{
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
