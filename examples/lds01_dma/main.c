#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#include "event.h"
#include "shell.h"

#include "lds01.h"
#include "lds01_params.h"

#ifdef MODULE_SHMEM
#include "shmem.h"
#endif

/* Device to use (defined in lds01_params.h) */
lds01_t lds01 = 0;

#define SHELL_BUFSIZE (128U)
#define FRAME_READER_PRIO (THREAD_PRIORITY_MAIN - 1)

/* Frame updater PID and stack */
static kernel_pid_t frame_updater_thread_pid;
static char frame_updater_thread_stack[THREAD_STACKSIZE_MAIN];

/* Event and event queue used to wake up the frame reader thread */
static event_t new_frame_event;
static event_queue_t new_frame_queue;

/* Local buffers */
uint16_t distances[LDS01_NB_ANGLES];
uint16_t intensities[LDS01_NB_ANGLES];

/**
 * @brief New frame available callback
 *
 * Function called by the driver each time a new frame is available.
 *
 * Post en event to wake up the thread in charge of lds01 data update.
 */
void new_frame_available_cb(void)
{
    event_post(&new_frame_queue, &new_frame_event);
}

/**
 * @brief LDS01 data updater function
 *
 * Function waiting for a new LDS01 data frame,
 * then calls the driver function to update the data arrays.
 *
 * @param[in]   args    Unused
 *
 * @return              0, exit only on thread exit
 */
static void *frame_updater_thread(void *arg)
{
    (void)arg;
    event_t *event;

    event_queue_init(&new_frame_queue);

    while ((event = event_wait(&new_frame_queue))) {
        lds01_update_last_frame(lds01);
    }

    return NULL;
}

/**
 * @brief LDS01 start command
 *
 * Reset LDS01 data and start it.
 *
 * @param[in]   argc    Unused
 * @param[in]   argv    Unused
 *
 * @return              0
 */
static int cmd_start(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    lds01_start(lds01);

    return 0;
}

/**
 * @brief LDS01 stop command
 *
 * @param[in]   argc    Unused
 * @param[in]   argv    Unused
 *
 * @return              0
 */
static int cmd_stop(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    lds01_stop(lds01);

    return 0;
}

/**
 * @brief Set distance filter command
 *
 * @param[in]   argc    Number of arguments
 * @param[in]   argv    Arguments list
 *
 * @return              0
 */
static int cmd_set_distance_filter(int argc, char **argv)
{
    if (argc < 2) {
        printf("usage: %s <filter_value>\n", argv[0]);
        return 1;
    }

    uint32_t new_filter = strtoul(argv[1], NULL, 0);

    lds01_set_distance_filter(lds01, new_filter);

    return 0;
}

/**
 * @brief Set intensity threshold command
 *
 * @param[in]   argc    Number of arguments
 * @param[in]   argv    Arguments list
 *
 * @return              0
 */
static int cmd_set_intensity_threshold(int argc, char **argv)
{
    if (argc < 2) {
        printf("usage: %s <threshold_value>\n", argv[0]);
        return 1;
    }

    uint32_t new_threshold = strtoul(argv[1], NULL, 0);

    lds01_set_min_intensity(lds01, new_threshold);

    return 0;
}

/**
 * @brief Print data command
 *
 * Print current distances and intensities in JSON format.
 *
 * @param[in]   argc    Unused
 * @param[in]   argv    Unused
 *
 * @return              0
 */
static int cmd_data(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    /* Make a local copy to reduce the lock time */
    lds01_get_distances(lds01, distances);
    lds01_get_intensities(lds01, intensities);

    printf("{\"distances\":[");
    for (uint32_t i = 0; i < LDS01_NB_ANGLES; i++) {
        if (i > 0) {
            printf(",");
        }
        printf("%u", distances[i]);
    }
    printf("],\"intensities\":[");
    for (uint32_t i = 0; i < LDS01_NB_ANGLES; i++) {
        if (i > 0) {
            printf(",");
        }
        printf("%u", intensities[i]);
    }
    printf("]}\n");

    return 0;
}

/* Shell commands and callbacks */
static const shell_command_t shell_commands[] = {
    { "start", "Start LDS01 device", cmd_start },
    { "stop", "Stop LDS01 device", cmd_stop },
    { "filter", "Set the filter value (max distance)", cmd_set_distance_filter },
    { "intensity_threshold", "Set the intensity threshold", cmd_set_intensity_threshold },
    { "data", "Print current values", cmd_data },
#ifdef MODULE_SHMEM
    SHMEM_SET_KEY_CMD,
#endif
    { NULL, NULL, NULL }
};

int main(void)
{
    puts("\n== LDS01 example using DMA driver ==");

    /* Start the frame reader thread */
    frame_updater_thread_pid = thread_create(
        frame_updater_thread_stack, sizeof(frame_updater_thread_stack),
        FRAME_READER_PRIO, 0, frame_updater_thread, NULL, "frame_reader");

    lds01_init(lds01, &lds01_params[0]);

    /* Run the shell */
    char line_buf[SHELL_BUFSIZE];

    shell_run(shell_commands, line_buf, SHELL_BUFSIZE);
    return 0;
}
