/* System includes */
#include <stdlib.h>

/* RIOT includes */
#include "xtimer.h"

/* Application includes */
#include "trace_utils.h"
#include "tracefd.h"
#include "platform.h"
#include "obstacles.h"

/* Trace filename */
#define TRACE_FILE "trace.txt"

/* Trace file descriptor on sdcard */
static tracefd_t tracefd_sdcard;

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

    for (;;) {
        xtimer_ticks32_t loop_start_time = xtimer_now();

        if (pf_trace_on()) {
            pf_print_state(tracefd_stdout);
        }
        pf_print_state(tracefd_sdcard);

        xtimer_periodic_wakeup(&loop_start_time, TASK_PERIOD_MS * US_PER_MS);
    }

    return NULL;
}

void trace_start(void)
{
    tracefd_sdcard = tracefd_new(TRACE_FILE);
    tracefd_open(tracefd_sdcard);

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
