#include "motion_control_common/BaseControllerEngine.hpp"
#include "motion_control_common/Controller.hpp"
#include "thread/thread.hpp"
#include "log.h"

#define ENABLE_DEBUG 0
#include <debug.h>

// RIOT includes
#include <time_units.h>
#include <ztimer.h>
#include <cstring>

namespace cogip {

namespace motion_control {

#define CONTROLLER_PRIO (THREAD_PRIORITY_MAIN - 2)

BaseControllerEngine::BaseControllerEngine(uint32_t engine_thread_period_ms) :
    enable_(true),
    controller_(nullptr),
    current_cycle_(0),
    pose_reached_(moving),
    timeout_cycle_counter_(0),
    timeout_ms_(0),
    timeout_enable_(false),
    engine_thread_period_ms_(engine_thread_period_ms)
{
    memset(engine_thread_stack_, 0, sizeof(engine_thread_stack_));
    mutex_init(&mutex_);
}

static void *_start_thread(void *arg)
{
    DEBUG("Engine start thread\n");
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

        DEBUG("Engine loop\n");

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

                LOG_ERROR("Engine timed out\n");

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
