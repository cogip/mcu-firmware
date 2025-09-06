#include "motion_control_common/BaseControllerEngine.hpp"
#include "motion_control_common/Controller.hpp"
#include <iostream>
#include "thread/thread.hpp"

// RIOT includes
#include <time_units.h>
#include <ztimer.h>

namespace cogip {

namespace motion_control {

#define CONTROLLER_PRIO (THREAD_PRIORITY_MAIN - 2)

static void *_start_thread(void *arg)
{
    COGIP_DEBUG_COUT("Engine start thread");
    static_cast<BaseControllerEngine *>(arg)->thread_loop();
    return EXIT_SUCCESS;
}

void BaseControllerEngine::set_controller(BaseController *controller) {
    controller_ = controller;
}

void BaseControllerEngine::thread_loop() {
    // Init loop iteration start time
    ztimer_now_t loop_start_time = ztimer_now(ZTIMER_USEC);

    while (true) {
        // Protect engine loop
        mutex_lock(&mutex_);

        COGIP_DEBUG_COUT("Engine loop");

        // Reset all read-only markers to allow data update
        io_.reset_readonly_markers();

        // Set controller inputs
        prepare_inputs();

        // Clear modified flags
        io_.clear_modified();

        if ((enable_) && (controller_)) {

            // Execute controller
            controller_->execute(io_);

            // Next cycle
            current_cycle_++;

            // Consider pose reached on timeout
            if ((timeout_enable_) && (--timeout_cycle_counter_ <= 0)) {
                // Reset timeout cycles counter
                timeout_cycle_counter_ = timeout_ms_ / engine_thread_period_ms_;
                // Force target pose status to notify the platform the timeout is over
                pose_reached_ = target_pose_status_t::timeout;

                std::cerr << "Engine timed out" << std::endl;

                enable_ = false;
            }

            // Process controller outputs
            process_outputs();
        }

        // End of engine loop
        mutex_unlock(&mutex_);

        // Wait thread period to end
        thread::thread_ztimer_periodic_wakeup(ZTIMER_USEC, &loop_start_time, engine_thread_period_ms_ * US_PER_MS);
    }
}

void BaseControllerEngine::start_thread() {
    thread_create(
        engine_thread_stack_,
        sizeof(engine_thread_stack_),
        CONTROLLER_PRIO,
        THREAD_CREATE_STACKTEST,
        _start_thread,
        this,
        "Controller thread"
        );
}

} // namespace motion_control

} // namespace cogip
