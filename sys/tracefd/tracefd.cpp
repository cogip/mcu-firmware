// Module includes
#include "tracefd/tracefd.hpp"
#include "tracefd_private.hpp"

// System includes
#include <cstdarg>

// RIOT includes
#include "riot/chrono.hpp"
#include "riot/thread.hpp"

namespace cogip {

namespace tracefd {

File out(stdout);
File err(stderr);

std::vector<File *> all_files;
static bool initialized = false;
static riot::mutex thread_lock;

// Files flusher thread PID
kernel_pid_t files_flusher_pid = -1;

// Files flusher thread stack
static char files_flusher_thread_stack[THREAD_STACKSIZE_DEFAULT];

static void *_thread_files_flusher(void *arg)
{
    (void)arg;
    while (true) {
        thread_lock.lock();
        thread_lock.unlock();
        flush_all();
        riot::this_thread::sleep_for(std::chrono::milliseconds(TRACEFD_FLUSH_INTERVAL));
    }
    return EXIT_SUCCESS;
}

void initialize_tracefd(void)
{
    if (initialized) {
        return;
    }

    if (init_root_dir()) {
        files_flusher_pid = thread_create(
            files_flusher_thread_stack,
            sizeof(files_flusher_thread_stack),
            TRACEFD_THREAD_PRIORITY,
            THREAD_CREATE_SLEEPING,
            _thread_files_flusher,
            NULL,
            "Trace file flusher"
            );
    }

    initialized = true;
    start_files_flusher();
}

void flush_all(void)
{
    for (auto trace_file: all_files) {
        trace_file->flush();
    }
}

void start_files_flusher(void)
{
    thread_lock.unlock();
}

void stop_files_flusher(void)
{
    thread_lock.lock();
}

} // namespace tracefd

} // namespace cogip
