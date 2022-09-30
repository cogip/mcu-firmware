// System includes
#include <malloc.h>
#include <iostream>

// RIOT includes
#include <etl/map.h>
#include <etl/pool.h>
#include <thread.h>
#include <time_units.h>
#include <ztimer.h>

// Project includes
#include "heap_private.hpp"
#include "PB_Sysmon.hpp"
#include "sysmon/sysmon.hpp"
#include "sysmon/ThreadStatus.hpp"
#ifdef MODULE_UARTPB
#include "uartpb/UartProtobuf.hpp"
#endif

// Periodic task
#define TASK_PERIOD_SEC    (1)

#ifdef MODULE_UARTPB
inline constexpr cogip::uartpb::uuid_t sysmon_uuid = 1509576639;
#endif

/// Start of the heapd
extern char *__sheap;

/// End of the heap
extern char *__eheap;

namespace cogip {

namespace sysmon {

using PB_Sysmon_Message = PB_Sysmon<MAXTHREADS, SYSMON_THREADSTATUS_NAME_MAX_LENGTH>;
// Sysmon Protobuf message
static PB_Sysmon_Message  pb_sysmon_message_;


#ifdef MODULE_UARTPB
// Protobuf serial interface
inline static cogip::uartpb::UartProtobuf *uart_protobuf = nullptr;
#endif


static etl::pool<ThreadStatus, MAXTHREADS> threads_status_pool;
static etl::map<kernel_pid_t, ThreadStatus *, MAXTHREADS> _sysmon_threads_status;
static MemoryStatus _sysmon_heap_status;

// Thread stack
static char _sysmon_updater_thread_stack[THREAD_STACKSIZE_SMALL];

// Mutex to avoid concurrent access to status
static mutex_t _mutex_sysmon = MUTEX_INIT;

// Update heap memory status
static void _update_heap_status(void)
{
    // Heap description
    static  const sysmon_heap_t heap = {
        .start = __sheap,
        .end = __eheap
    };

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

    std::cout << "  heap size = " << _sysmon_heap_status.size() << " bytes" << std::endl;
    std::cout << "  heap used = " << _sysmon_heap_status.used() << " bytes" << std::endl;
    std::cout << std::endl;

    mutex_unlock(&_mutex_sysmon);
}

/// Update threads stack and scheduling status
static void _update_threads_status(void)
{
    for (kernel_pid_t i = KERNEL_PID_FIRST; i <= KERNEL_PID_LAST; i++) {
        thread_t *thread = thread_get(i);

        if (thread != NULL) {
            mutex_lock(&_mutex_sysmon);

            if (! _sysmon_threads_status[i]) {
                _sysmon_threads_status[i] = threads_status_pool.create();
            }

            // Convert thread state to a string
            _sysmon_threads_status[i]->set_name(thread->name);
            // Thread stack total size
            _sysmon_threads_status[i]->set_size(thread_get_stacksize(thread));
            // Thread used stack size
            _sysmon_threads_status[i]->set_used(
                _sysmon_threads_status[i]->size()
                - thread_measure_stack_free((const char*)thread_get_stackstart(thread))
                );

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

    if (! _sysmon_threads_status[pid]) {
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
        thread_t *thread = thread_get(i);

        if (thread != NULL) {
            mutex_lock(&_mutex_sysmon);

            std::cout << "Thread '" << thread->name << "' with pid " << i << std::endl;
            std::cout << "  stack size = " <<_sysmon_threads_status[i]->size() << " bytes" << std::endl;
            std::cout << "  stack used = " <<_sysmon_threads_status[i]->used() << " bytes" << std::endl;
            std::cout << "  loops      = " <<_sysmon_threads_status[i]->loops() << std::endl;
            std::cout << "  overshots  = " <<_sysmon_threads_status[i]->overshots() << std::endl;

            mutex_unlock(&_mutex_sysmon);
        }
    }

    std::cout << std::endl;
}

#ifdef MODULE_UARTPB
void register_uartpb(cogip::uartpb::UartProtobuf *uartpb_ptr)
{
    uart_protobuf = uartpb_ptr;
}

static void _uartpb_send_status(void)
{
    if (uart_protobuf) {
        uart_protobuf->send_message(sysmon_uuid, &pb_sysmon_message_);
    }
    else {
        std::cerr << "sysmon: error, uartpb interface has not been registered." << std::endl;
    }
}
#endif // MODULE_UARTPB

static void *_thread_status_updater(void *data)
{
    (void)data;

    ztimer_now_t loop_start_time = ztimer_now(ZTIMER_USEC);

    while (true) {
        pb_sysmon_message_.clear();
        _update_heap_status();
        _update_threads_status();
#ifdef MODULE_UARTPB
        _uartpb_send_status();
#endif

        ztimer_periodic_wakeup(ZTIMER_SEC, &loop_start_time, TASK_PERIOD_SEC);
    }

    return 0;
}

void sysmon_start(void)
{
    thread_create(
        _sysmon_updater_thread_stack,
        sizeof(_sysmon_updater_thread_stack),
        THREAD_PRIORITY_MAIN - 1,
        THREAD_CREATE_STACKTEST,
        _thread_status_updater,
        NULL,
        "System monitor"
        );
}

} // namespace sysmon

} // namespace cogip

/// @}
