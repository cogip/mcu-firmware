/* System includes */
#include <stdio.h>
#include <stdlib.h>

/* RIOT includes */
#include "xtimer.h"

/* Project includes */
#include "tracefd.h"

int main(void)
{
    tracefd_t tracefd;

    tracefd_printf(tracefd_stdout, "\n== Trace file descriptor example ==\n");

    tracefd_printf(tracefd_stdout, "Example 1: write in a file and close it\n");
    tracefd = tracefd_new("example1.txt");
    tracefd_open(tracefd);
    tracefd_printf(tracefd, "Trace example 1\n");
    tracefd_close(tracefd);

    tracefd_printf(tracefd_stdout, "Example 2: write in a file and do not close it\n");
    tracefd = tracefd_new("example2.txt");
    tracefd_open(tracefd);
    tracefd_printf(tracefd, "Trace example 2\n");
    tracefd_printf(tracefd_stdout, "  Wait 2s to be sure the files flusher thread has operated.\n");
    xtimer_sleep(2);

    tracefd_printf(tracefd_stdout, "Stop files flusher thread\n");
    tracefd_stop_files_flusher();

    tracefd_printf(tracefd_stdout, "Example 3: write in a file and do not close it\n");
    tracefd = tracefd_new("example3.txt");
    tracefd_open(tracefd);
    tracefd_printf(tracefd, "Trace example 3\n");

    tracefd_printf(tracefd_stdout, "== End of example ==\n");

    tracefd_printf(tracefd_stdout, "On real hardware, without automatic flushing, 'example3.txt' should be empty.\n\n");

    return 0;
}
