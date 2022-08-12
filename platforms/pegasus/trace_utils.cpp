#include "trace_utils.hpp"

// RIOT includes
#include <ztimer.h>

// Application includes
#include "platform.hpp"

/* Periodic task */
#define TASK_PERIOD_USEC    (500 * US_PER_MS)

/* Thread stack */
static char trace_thread_stack[THREAD_STACKSIZE_MEDIUM];

/* Thread priority */
#define TRACE_PRIO (THREAD_PRIORITY_MAIN - 1)

/* Thread loop */
static void *_thread_trace(void *arg)
{
    (void)arg;

    // Init loop iteration start time
    ztimer_now_t loop_start_time = ztimer_now(ZTIMER_USEC);

    while (true) {
        if (pf_trace_on()) {
            pf_send_pb_state();
        }

        // Wait thread period to end
        ztimer_periodic_wakeup(ZTIMER_USEC, &loop_start_time, TASK_PERIOD_USEC);
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
