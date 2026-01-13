// System includes
#include "log.h"
#include <inttypes.h>
#include <malloc.h>

// RIOT includes
#include <etl/map.h>
#include <etl/pool.h>
#include <thread.h>
#include <time_units.h>
#include <ztimer.h>

// Project includes
#include "PB_Sysmon.hpp"
#include "heap_private.hpp"
#include "sysmon/ThreadStatus.hpp"
#include "sysmon/sysmon.hpp"
#include "thread/thread.hpp"
#ifdef MODULE_CANPB
#include "canpb/CanProtobuf.hpp"
#endif

// Periodic task
#define TASK_PERIOD_SEC (1)

#ifdef MODULE_CANPB
inline constexpr cogip::canpb::uuid_t sysmon_uuid = 0xf001;
#endif

/// Start of the heapd
extern char* __sheap;

/// End of the heap
extern char* __eheap;

namespace cogip {

namespace sysmon {

using PB_Sysmon_Message = PB_Sysmon<MAXTHREADS, SYSMON_THREADSTATUS_NAME_MAX_LENGTH>;
// Sysmon Protobuf message
static PB_Sysmon_Message pb_sysmon_message_;

#ifdef MODULE_CANPB
// Protobuf serial interface
inline static cogip::canpb::CanProtobuf* can_protobuf = nullptr;
#endif

static etl::pool<ThreadStatus, MAXTHREADS> threads_status_pool;
static etl::map<kernel_pid_t, ThreadStatus*, MAXTHREADS> _sysmon_threads_status;
static MemoryStatus _sysmon_heap_status;

// Thread stack
static char _sysmon_updater_thread_stack[THREAD_STACKSIZE_SMALL];

// Mutex to avoid concurrent access to status
static mutex_t _mutex_sysmon = MUTEX_INIT;

// Update heap memory status
static void _update_heap_status(void)
{
    // Heap description
    static const sysmon_heap_t heap = {.start = __sheap, .end = __eheap};

    // Allocation status
    MALLINFO minfo = MALLINFO_FUNC();

    mutex_lock(&_mutex_sysmon);

    // Heap total size
    _sysmon_heap_status.set_size(heap.end - heap.start);
    // Used heap size
    _sysmon_heap_status.set_used(minfo.uordblks);

    // Update heap status Protobuf message
    _sysmon_heap_status.update_pb_message();
    // Add heap status message to sysmon overall Protobuf message
    pb_sysmon_message_.set_heap_status(_sysmon_heap_status.pb_message());

    mutex_unlock(&_mutex_sysmon);
}

void display_heap_status(void)
{
    mutex_lock(&_mutex_sysmon);

    LOG_INFO("  heap size = %" PRIu32 " bytes\n",
             static_cast<uint32_t>(_sysmon_heap_status.size()));
    LOG_INFO("  heap used = %" PRIu32 " bytes\n",
             static_cast<uint32_t>(_sysmon_heap_status.used()));

    mutex_unlock(&_mutex_sysmon);
}

/// Update threads stack and scheduling status
static void _update_threads_status(void)
{
    for (kernel_pid_t i = KERNEL_PID_FIRST; i <= KERNEL_PID_LAST; i++) {
        const thread_t* thread = thread_get(i);

        if (thread != NULL) {
            mutex_lock(&_mutex_sysmon);

            if (!_sysmon_threads_status[i]) {
                _sysmon_threads_status[i] = threads_status_pool.create();
            }

            // Convert thread state to a string
            _sysmon_threads_status[i]->set_name(thread->name);
            // Thread stack total size
            _sysmon_threads_status[i]->set_size(thread_get_stacksize(thread));
            // Thread used stack size
            _sysmon_threads_status[i]->set_used(_sysmon_threads_status[i]->size() -
                                                thread_measure_stack_free(thread));

            // Update thread status Protobuf message
            _sysmon_threads_status[i]->update_pb_message();
            // Add thread status message to sysmon overall Protobuf message
            pb_sysmon_message_.add_threads_status(_sysmon_threads_status[i]->pb_message());

            mutex_unlock(&_mutex_sysmon);
        }
    }
}

void update_thread_sched_status(kernel_pid_t pid, bool has_overshot)
{
    mutex_lock(&_mutex_sysmon);

    if (!_sysmon_threads_status[pid]) {
        _sysmon_threads_status[pid] = threads_status_pool.create();
    }

    _sysmon_threads_status[pid]->inc_loops();

    if (has_overshot) {
        _sysmon_threads_status[pid]->inc_overshots();
    }

    mutex_unlock(&_mutex_sysmon);
}

void display_threads_status(void)
{
    for (kernel_pid_t i = KERNEL_PID_FIRST; i <= KERNEL_PID_LAST; i++) {
        const thread_t* thread = thread_get(i);

        if (thread != NULL) {
            mutex_lock(&_mutex_sysmon);

            LOG_INFO("Thread '%s' with pid %" PRIi16 "\n", thread->name, static_cast<int16_t>(i));
            LOG_INFO("  stack size = %" PRIu32 " bytes\n",
                     static_cast<uint32_t>(_sysmon_threads_status[i]->size()));
            LOG_INFO("  stack used = %" PRIu32 " bytes\n",
                     static_cast<uint32_t>(_sysmon_threads_status[i]->used()));
            LOG_INFO("  loops      = %" PRIu32 "\n",
                     static_cast<uint32_t>(_sysmon_threads_status[i]->loops()));
            LOG_INFO("  overshots  = %" PRIu32 "\n",
                     static_cast<uint32_t>(_sysmon_threads_status[i]->overshots()));

            mutex_unlock(&_mutex_sysmon);
        }
    }
}

void update_status(void)
{
    _update_heap_status();
    _update_threads_status();
}

#ifdef MODULE_CANPB
void register_canpb(cogip::canpb::CanProtobuf* canpb_ptr)
{
    can_protobuf = canpb_ptr;
}

static void _canpb_send_status(void)
{
    if (can_protobuf) {
        can_protobuf->send_message(sysmon_uuid, &pb_sysmon_message_);
    } else {
        LOG_ERROR("sysmon: canpb interface has not been registered\n");
    }
}
#endif // MODULE_CANPB

static void* _thread_status_updater(void* data)
{
    (void)data;

    ztimer_now_t loop_start_time = ztimer_now(ZTIMER_SEC);

    while (true) {
        pb_sysmon_message_.clear();
        _update_heap_status();
        _update_threads_status();
#ifdef MODULE_CANPB
        _canpb_send_status();
#endif
        cogip::thread::thread_ztimer_periodic_wakeup(ZTIMER_SEC, &loop_start_time, TASK_PERIOD_SEC);
    }

    return 0;
}

void sysmon_start(void)
{
    thread_create(_sysmon_updater_thread_stack, sizeof(_sysmon_updater_thread_stack),
                  THREAD_PRIORITY_MAIN - 1, THREAD_CREATE_STACKTEST, _thread_status_updater, NULL,
                  "System monitor");
}

} // namespace sysmon

} // namespace cogip

/// @}
