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

    puts("\n== Trace file descriptor example ==");

    puts("Example 1: write in a file and close it");
    tracefd = tracefd_init("example1.txt");
    tracefd_open(tracefd);
    tracefd_printf(tracefd, "Trace example 1\n");
    tracefd_close(tracefd);

    puts("Example 2: write in a file and do not close it");
    tracefd = tracefd_init("example2.txt");
    tracefd_open(tracefd);
    tracefd_printf(tracefd, "Trace example 2\n");
    puts("  Wait 2s to be sure the files flusher thread has operated.");
    xtimer_sleep(2);

    puts("Stop files flusher thread");
    tracefd_stop_files_flusher();

    puts("Example 3: write in a file and do not close it");
    tracefd = tracefd_init("example3.txt");
    tracefd_open(tracefd);
    tracefd_printf(tracefd, "Trace example 3\n");

    puts("== End of example ==");

    puts("On real hardware, without automatic flushing, 'example3.txt' should be empty.\n");

    return 0;
}
