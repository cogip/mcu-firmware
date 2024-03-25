#include "motion_control_common/BaseControllerEngine.hpp"
#include "motion_control_common/Controller.hpp"
#include "thread/thread.hpp"

// RIOT includes
#include <time_units.h>
#include <ztimer.h>

#ifndef CONTROLLER_PERIOD_USEC
    #define CONTROLLER_PERIOD_USEC (20 * US_PER_MS)
#endif

namespace cogip {

namespace motion_control {

static char controller_thread_stack[THREAD_STACKSIZE_LARGE];
#define CONTROLLER_PRIO (THREAD_PRIORITY_MAIN - 1)

static void *_start_thread(void *arg)
{
    COGIP_DEBUG_COUT("Engine start thread");
    ((BaseControllerEngine *)arg)->thread_loop();
    return EXIT_SUCCESS;
}

void BaseControllerEngine::set_controller(BaseController *controller) {
    controller_ = controller;
}

void BaseControllerEngine::thread_loop() {
    // Init loop iteration start time
    ztimer_now_t loop_start_time = ztimer_now(ZTIMER_USEC);

    while (true) {
        COGIP_DEBUG_COUT("Engine loop");

        if ((enable_) && (controller_)) {

            // Set controller inputs
            prepare_inputs();

            // Execute controller only if target pose is not reached
            if (pose_reached_ != target_pose_status_t::reached) {
                // Cycles decrementing counter
                static uint32_t timeout_cycle_counter = timeout_cycle_number_;

                // Execute controller
                if (controller_) {
                    controller_->execute();
                }

                // Next cycle
                current_cycle_++;

                // Consider pose reached on timeout
                if ((timeout_enable_) && (!--timeout_cycle_counter)) {
                    // Reset timeout cycles counter
                    timeout_cycle_counter = timeout_cycle_number_;
                    // Force target pose status to notify the platform the timeout is over
                    pose_reached_ = target_pose_status_t::reached;
                }
            }

            // Process controller outputs
            process_outputs();
        }

        // Wait thread period to end
        thread::thread_ztimer_periodic_wakeup(ZTIMER_USEC, &loop_start_time, CONTROLLER_PERIOD_USEC);
    }
}

void BaseControllerEngine::start_thread() {
    thread_create(
        controller_thread_stack,
        sizeof(controller_thread_stack),
        CONTROLLER_PRIO,
        THREAD_CREATE_STACKTEST,
        _start_thread,
        this,
        "Controller thread"
        );
}

} // namespace motion_control

} // namespace cogip
