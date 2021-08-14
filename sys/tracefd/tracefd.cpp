// Module includes
#include "tracefd.hpp"
#include "tracefd_private.hpp"

// System includes
#include <cstdarg>

// RIOT includes
#include "riot/chrono.hpp"
#include "riot/thread.hpp"

namespace cogip {

namespace tracefd {

file out(stdout);
file err(stderr);

static bool initialized = false;
static std::vector<file *> all_files;
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
        cogip::tracefd::flush_all();
        riot::this_thread::sleep_for(std::chrono::milliseconds(TRACEFD_FLUSH_INTERVAL));
    }
    return EXIT_SUCCESS;
}

static void _initialize(void)
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

file::file(const std::string & filename)
{
    if (! initialized) {
        _initialize();
    }

    filename_ = std::string(TRACEFD_ROOT_DIR) + "/" + filename;
    // Create or truncate the file
    file_ = fopen(filename_.c_str(), "w");
    if (! file_) {
        throw std::runtime_error("Can't open file");
    }

    fclose(file_);
    file_ = nullptr;

    all_files.push_back(this);
}

file::file(FILE *f) : file_(f), filename_("buitin")
{
}

void file::open(void)
{
    if (file_ == nullptr) {
        file_ = fopen(filename_.c_str(), "a");
    }
}

void file::close(void)
{
    if (file_) {
        fclose(file_);
        file_ = nullptr;
    }
}

void file::lock(void)
{
    mutex_.lock();
}

void file::unlock(void)
{
    mutex_.unlock();
}

void file::printf(const char *format, ...)
{
    assert(file_);

    va_list argp;
    va_start(argp, format);
    vfprintf(file_, format, argp);
    va_end(argp);
    fflush(file_);
}

void file::logf(const char *format, ...)
{
    assert(file_);

    lock();

    fprintf(file_, "{\"log\": \"");
    fflush(file_);

    va_list argp;
    va_start(argp, format);
    vfprintf(file_, format, argp);
    va_end(argp);
    fflush(file_);

    fprintf(file_, "\"}\n");
    fflush(file_);

    unlock();
}

void file::flush(void)
{
    lock();
    if (file_) {
        close();
    }
    open();
    unlock();
}

} // namespace tracefd

} // namespace cogip
