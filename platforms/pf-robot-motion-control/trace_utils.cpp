#include "trace_utils.hpp"

// RIOT includes
#include <ztimer.h>

// Application includes
#include "motion_control.hpp"
#include "platform.hpp"
#include "thread/thread.hpp"

/* Periodic task */
#define TASK_PERIOD_MSEC_POSE (50)
#define TASK_PERIOD_MSEC_STATE (200)
#define TASK_PERIOD_MSEC_TELEMETRY (50)

/* Thread stack */
static char pose_thread_stack[THREAD_STACKSIZE_MEDIUM];
static char state_thread_stack[THREAD_STACKSIZE_MEDIUM];
static char telemetry_thread_stack[THREAD_STACKSIZE_MEDIUM];

/* Thread priority */
#define TRACE_PRIO (THREAD_PRIORITY_MAIN - 1)

/* Thread loop */
static void* _thread_pose(void* arg)
{
    (void)arg;

    // Init loop iteration start time
    ztimer_now_t loop_start_time = ztimer_now(ZTIMER_MSEC);

    while (true) {
        if (pf_trace_on()) {
            cogip::pf::motion_control::pf_send_pb_pose();
        }

        // Wait thread period to end
        cogip::thread::thread_ztimer_periodic_wakeup(ZTIMER_MSEC, &loop_start_time,
                                                     TASK_PERIOD_MSEC_POSE);
    }

    return NULL;
}

static void* _thread_state(void* arg)
{
    (void)arg;

    // Init loop iteration start time
    ztimer_now_t loop_start_time = ztimer_now(ZTIMER_MSEC);

    while (true) {
        if (pf_trace_on()) {
            cogip::pf::motion_control::pf_send_pb_state();
        }

        // Wait thread period to end
        cogip::thread::thread_ztimer_periodic_wakeup(ZTIMER_MSEC, &loop_start_time,
                                                     TASK_PERIOD_MSEC_STATE);
    }

    return NULL;
}

static void* _thread_telemetry(void* arg)
{
    (void)arg;

    // Init loop iteration start time
    ztimer_now_t loop_start_time = ztimer_now(ZTIMER_MSEC);

    while (true) {
        cogip::pf::motion_control::pf_send_encoder_telemetry();

        // Wait thread period to end
        cogip::thread::thread_ztimer_periodic_wakeup(ZTIMER_MSEC, &loop_start_time,
                                                     TASK_PERIOD_MSEC_TELEMETRY);
    }

    return NULL;
}

void trace_start(void)
{
    /* Start the state thread */
    thread_create(state_thread_stack, sizeof(state_thread_stack), TRACE_PRIO,
                  THREAD_CREATE_STACKTEST, _thread_state, NULL, "State thread");

    /* Start the pose thread */
    thread_create(pose_thread_stack, sizeof(pose_thread_stack), TRACE_PRIO, THREAD_CREATE_STACKTEST,
                  _thread_pose, NULL, "Pose thread");

    /* Start the telemetry thread */
    thread_create(telemetry_thread_stack, sizeof(telemetry_thread_stack), TRACE_PRIO,
                  THREAD_CREATE_STACKTEST, _thread_telemetry, NULL, "Telemetry thread");
}
