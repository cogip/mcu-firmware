// Module includes
#include "tracefd/tracefd.hpp"
#include "tracefd/File.hpp"
#include "tracefd_private.hpp"

// System includes
#include <cstdarg>

// RIOT includes
#include "riot/chrono.hpp"
#include "riot/thread.hpp"

namespace cogip {

namespace tracefd {

File::File(const std::string & filename)
{
    initialize_tracefd();

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

File::File(FILE *f) : file_(f), filename_("buitin")
{
}

void File::open(void)
{
    if (file_ == nullptr) {
        file_ = fopen(filename_.c_str(), "a");
    }
}

void File::close(void)
{
    if (file_) {
        fclose(file_);
        file_ = nullptr;
    }
}

void File::lock(void)
{
    mutex_.lock();
}

void File::unlock(void)
{
    mutex_.unlock();
}

void File::printf(const char *format, ...)
{
    assert(file_);

    va_list argp;
    va_start(argp, format);
    vfprintf(file_, format, argp);
    va_end(argp);
    fflush(file_);
}

void File::logf(const char *format, ...)
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

void File::flush(void)
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
