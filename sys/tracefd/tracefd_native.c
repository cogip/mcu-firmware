/* System includes */
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

/* Module includes */
#include "tracefd.h"
#include "tracefd_private.h"

void tracefd_init_root_dir(void)
{
    struct stat st;

    if (stat(TRACEFD_ROOT_DIR, &st) == -1) {
        mkdir(TRACEFD_ROOT_DIR, 0700);
    }
}
