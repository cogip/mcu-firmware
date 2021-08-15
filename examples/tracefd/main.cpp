// RIOT includes
#include "riot/chrono.hpp"
#include "riot/thread.hpp"

// Project includes
#include "tracefd/tracefd.hpp"

int main(void)
{
    cogip::tracefd::out.printf("\n== Trace file descriptor example ==\n");

    cogip::tracefd::out.printf("Example 1: write in a file and close it\n");
    std::string filename1 = "example1.txt";
    cogip::tracefd::File *tracefd1;
    try {
        tracefd1 = new cogip::tracefd::File(filename1);
    }
    catch(std::runtime_error &e) {
        tracefd1 = nullptr;
    }
    // if/else statements are separated from try/catch to simulate that the files
    // can be initialized in one file, but used somewhere else.
    if (tracefd1) {
        tracefd1->open();
        tracefd1->printf("Trace example 1\n");
        tracefd1->close();
    }
    else {
        cogip::tracefd::out.printf("Example 1 failed\n");
    }

    cogip::tracefd::out.printf("Example 2: write in a file and do not close it\n");
    try {
        cogip::tracefd::File tracefd2("example2.txt");
        tracefd2.open();
        tracefd2.printf("Trace example 2\n");
    }
    catch(...) {
        cogip::tracefd::out.printf("Example 2 failed\n");
    }

    cogip::tracefd::out.printf("Wait 2s to be sure the files flusher thread has operated.\n");
    riot::this_thread::sleep_for(std::chrono::seconds(2));

    cogip::tracefd::out.printf("Stop files flusher thread\n");
    cogip::tracefd::stop_files_flusher();

    try {
        cogip::tracefd::out.printf("Example 3: write in a file and do not close it\n");
        cogip::tracefd::File tracefd3("example3.txt");
        tracefd3.open();
        tracefd3.printf("Trace example 3\n");
    }
    catch(...) {
        cogip::tracefd::out.printf("Example 3 failed\n");
    }

    cogip::tracefd::out.printf("== End of example ==\n");

    cogip::tracefd::out.printf("On real hardware, without automatic flushing, 'example3.txt' should be empty.\n\n");

    return 0;
}
