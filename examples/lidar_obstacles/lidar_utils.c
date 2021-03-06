/* System includes */
#include <stdio.h>
#include <stdlib.h>

/* RIOT includes */
#include "event.h"
#include "xtimer.h"

/* Project includes */
#include "lds01.h"
#include "lds01_params.h"

/* Application includes */
#include "lidar_utils.h"

/* Lidar device to use (defined in lds01_params.h) */
lds01_t lds01 = 0;

/* Event and event queue used to wake up the Lidar frame reader thread */
static event_t new_frame_event;
static event_queue_t new_frame_queue;

/* Thread stack */
static char lidar_frame_updater_thread_stack[THREAD_STACKSIZE_LARGE];

/* Thread priority */
#define LIDAR_FRAME_UPDATER_PRIO (THREAD_PRIORITY_MAIN - 1)

/* Thread loop */
static void *_thread_lidar_frame_updater(void *arg)
{
    (void)arg;
    event_t *event;
    event_queue_init(&new_frame_queue);

    while ((event = event_wait(&new_frame_queue))) {
        lds01_update_last_frame(lds01);
    }

    return NULL;
}

/* Callback for LDS01 DMA driver */
void new_frame_available_cb(void)
{
    event_post(&new_frame_queue, &new_frame_event);
}

/* Print data in JSON format */
static void _print_data(FILE *out, const uint16_t *distances)
{
    fprintf(out, "{\"distances\":[");
    fflush(out);
    for (uint32_t i = 0; i < LDS01_NB_ANGLES; i++) {
        if (i > 0) {
            fprintf(out, ",");
            fflush(out);
        }
        fprintf(out, "%u", distances[i]);
        fflush(out);
    }
    fprintf(out, "]}\n");
    fflush(out);
}

/* Shell command to print data */
int lidar_cmd_print_data(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    uint16_t distances[LDS01_NB_ANGLES];
    lds01_get_distances(lds01, distances);

    _print_data(stdout, distances);

    return EXIT_SUCCESS;
}

lds01_t lidar_get_device(void)
{
    return lds01;
}

void lidar_start(uint16_t max_distance, uint16_t min_intensity)
{
    /* Start the frame updater thread */
    thread_create(
        lidar_frame_updater_thread_stack,
        sizeof(lidar_frame_updater_thread_stack),
        LIDAR_FRAME_UPDATER_PRIO,
        0,
        _thread_lidar_frame_updater,
        NULL,
        "Lidar frame updater"
        );

    if (lds01_init(lds01, &lds01_params[0]) == 0) {
        xtimer_msleep(500);
        lds01_set_distance_filter(lds01, max_distance);
        lds01_set_min_intensity(lds01, min_intensity);
        lds01_start(lds01);
    }
    else {
        puts("Lidar initialisation failed.");
    }
}
