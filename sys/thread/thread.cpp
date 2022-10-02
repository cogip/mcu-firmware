// System includes
#include <iostream>

// RIOT includes
#include <irq.h>
#include <thread.h>
#include <ztimer.h>

// Project includes
#ifdef MODULE_SYSMON
#include "sysmon/sysmon.hpp"
#endif
#include "thread/thread.hpp"

namespace cogip {

namespace thread {

void thread_ztimer_periodic_wakeup(ztimer_clock_t *clock, uint32_t *last_wakeup, uint32_t period)
{
    unsigned state = irq_disable();
    uint32_t now = ztimer_now(clock);
    uint32_t target = *last_wakeup + period;
    int64_t offset = (int64_t)target - (int64_t)now;
#ifdef MODULE_SYSMON
    bool has_overshot = false;
#endif
    kernel_pid_t pid = thread_getpid();

    irq_restore(state);

    if (now <= target) {
        ztimer_sleep(clock, (uint32_t)offset);
        *last_wakeup = target;
    }
    else {
        *last_wakeup = now;
#ifdef MODULE_SYSMON
        has_overshot = true;
#endif
        std::cerr << "[WARNING] Thread '" << thread_getname(pid) << "' latency: " << offset << std::endl;
    }

#ifdef MODULE_SYSMON
    cogip::sysmon::update_thread_sched_status(pid, has_overshot);
#endif
}

} // namespace thread

} // namespace cogip

/// @}
