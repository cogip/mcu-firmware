// Module includes
#include "tracefd/tracefd.hpp"
#include "tracefd_private.hpp"

// System includes
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

namespace cogip {

namespace tracefd {

bool init_root_dir(void)
{
    struct stat st;
    int res = 0;

    res = stat(TRACEFD_ROOT_DIR, &st);
    if (res == -1) {
        res = mkdir(TRACEFD_ROOT_DIR, 0700);
    }
    return (res == 0);
}

} // namespace file

} // namespace cogip
