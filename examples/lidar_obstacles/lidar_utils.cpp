#include "lidar_utils.hpp"

// System includes
#include <cstdio>
#include <cstdlib>

// RIOT includes
#include "event.h"
#include "riot/chrono.hpp"
#include "riot/thread.hpp"

// Project includes
#include "lds01.h"
#include "lds01_params.h"
#include "tracefd.hpp"

// Lidar device to use (defined in lds01_params.h)
lds01_t lds01 = 0;

// Event and event queue used to wake up the Lidar frame reader thread
static event_t new_frame_event;
static event_queue_t new_frame_queue;

riot::thread *lidar_updater_thread = nullptr;

// Thread loop
static void _thread_lidar_frame_updater(void)
{
    event_t *event;
    event_queue_init(&new_frame_queue);

    while ((event = event_wait(&new_frame_queue))) {
        lds01_update_last_frame(lds01);
    }
}

// Callback for LDS01 DMA driver
void new_frame_available_cb(void)
{
    event_post(&new_frame_queue, &new_frame_event);
}

// Print data in JSON format
static void _print_data(cogip::tracefd::file &out, const uint16_t *distances)
{
    out.printf("{\"distances\":[");
    for (uint32_t i = 0; i < LDS01_NB_ANGLES; i++) {
        if (i > 0) {
            out.printf(",");
        }
        out.printf("%u", distances[i]);
    }
    out.printf("]}\n");
}

// Shell command to print data
int lidar_cmd_print_data(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    uint16_t distances[LDS01_NB_ANGLES];
    lds01_get_distances(lds01, distances);

    _print_data(cogip::tracefd::out, distances);

    return EXIT_SUCCESS;
}

lds01_t lidar_get_device(void)
{
    return lds01;
}

void lidar_start(uint16_t max_distance, uint16_t min_intensity)
{
    // Start the frame updater thread
    lidar_updater_thread = new riot::thread(_thread_lidar_frame_updater);

    if (lds01_init(lds01, &lds01_params[0]) == 0) {
        riot::this_thread::sleep_for(std::chrono::milliseconds(500));
        lds01_set_distance_filter(lds01, max_distance);
        lds01_set_min_intensity(lds01, min_intensity);
        lds01_start(lds01);
    }
    else {
        puts("Lidar initialisation failed.");
    }
}
