#include "lds01.h"
#include "lds01_params.h"

#define ENABLE_DEBUG (0)
#include "debug.h"

#include <string.h>

#define LDS01_NUM ARRAY_SIZE(lds01_params)  /**< number of used lds01 devices */
#define LDS01_FRAME_SYNC_BYTE (0xFA)        /**< first byte of a frame */
#define LDS01_NB_FRAMES (40U)               /**< number of frames to get all angles */
#define LDS01_NB_ANGLES_BY_FRAME (6U)       /**< number of angles in one frame */

/* Allocate memory for the device descriptor */
lds01_t lds01_devs[LDS01_NUM];

static void lds01_rx_cb(void *arg, uint8_t data)
{
    lds01_t *lds01 = (lds01_t *)arg;

    if (data == LDS01_FRAME_SYNC_BYTE && lds01->remaining_bytes_in_frame == 0) {
        lds01->remaining_bytes_in_frame = LDS01_FRAME_SIZE;
    }
    if (lds01->remaining_bytes_in_frame == 0) {
        return;
    }

    ringbuffer_add_one(&(lds01->rx_buf), data);
    lds01->remaining_bytes_in_frame--;

    if (lds01->remaining_bytes_in_frame == 0) {
        if (lds01->params.new_frame_cb) {
            lds01->params.new_frame_cb();
        }
        else {
            ringbuffer_remove(&(lds01->rx_buf), LDS01_FRAME_SIZE);
        }
    }
}

static void lds01_reset_data(lds01_t *lds01)
{
    lds01->remaining_bytes_in_frame = 0;
    memset((void *)lds01->distances, 0, LDS01_NB_ANGLES * sizeof(uint16_t));
    memset((void *)lds01->intensities, 0, LDS01_NB_ANGLES * sizeof(uint16_t));
    ringbuffer_init(&(lds01->rx_buf), lds01->rx_mem, LDS01_BUFFER_SIZE);
}

static int lds01_is_frame_valid(const lds01_frame_t *frame)
{
    int sum = 0;

    for (unsigned int i = 0; i < LDS01_NB_FRAMES; i++) {
        sum += ((const char *)frame)[i];
    }
    sum &= 0xFF;
    return frame->checksum[0] == 0xFF - sum;
}

static uint16_t lds01_get_filtered_distance(lds01_t *lds01, uint16_t distance, uint16_t intensity)
{
    if (lds01->filter == 0) {
        return distance;
    }
    if (intensity == 0 || distance == 0 || distance > lds01->filter) {
        return lds01->filter;
    }
    return distance;
}

void lds01_update_last_frame(lds01_t *lds01)
{
    lds01_frame_t frame;
    uint16_t distances_tmp[LDS01_NB_ANGLES_BY_FRAME];
    uint16_t intensities_tmp[LDS01_NB_ANGLES_BY_FRAME];
    unsigned bytes_read = ringbuffer_get(&(lds01->rx_buf), (char *)&frame, LDS01_FRAME_SIZE);

    if (bytes_read != LDS01_FRAME_SIZE) {
        printf("Read error: %u\n", bytes_read);
    }
    if (!lds01_is_frame_valid(&frame)) {
        return;
    }

    /* Compute new values in a local array */
    for (unsigned int i = 0; i < LDS01_NB_ANGLES_BY_FRAME; i++) {
        uint16_t distance = (frame.offsets[i].distance[1] << 8) + frame.offsets[i].distance[0];
        uint16_t intensity = (frame.offsets[i].intensity[1] << 8) + frame.offsets[i].intensity[0];
        distances_tmp[i] = lds01_get_filtered_distance(lds01, distance, intensity);
        intensities_tmp[i] = intensity;
    }

    uint8_t index = frame.index - 0xA0;

    /* Update values in arrays given in parameters*/
    mutex_lock(&lds01->data_lock);
    memcpy(
        (void *)(&lds01->distances[index * LDS01_NB_ANGLES_BY_FRAME]),
        (void *)distances_tmp, LDS01_NB_ANGLES_BY_FRAME * sizeof(uint16_t));
    memcpy(
        (void *)(&lds01->intensities[index * LDS01_NB_ANGLES_BY_FRAME]),
        (void *)intensities_tmp, LDS01_NB_ANGLES_BY_FRAME * sizeof(uint16_t));
    mutex_unlock(&lds01->data_lock);
}

int lds01_init(lds01_t *lds01, const lds01_params_t *params)
{
    lds01->params = *params;
    lds01->filter = 0;
    lds01_reset_data(lds01);
    mutex_init(&lds01->data_lock);

    int res = uart_init(lds01->params.uart, LDS01_UART_BAUD, lds01_rx_cb, (void *)lds01);
    if (res == UART_NOBAUD) {
        printf("Error: Given baudrate (%u) not possible\n", LDS01_UART_BAUD);
        return 1;
    }
    else if (res != UART_OK) {
        puts("Error: Unable to initialize UART device");
        return 1;
    }

    lds01->running = true; /* force stop on startup */
    lds01_stop(lds01);

    DEBUG("LDS01 UART initialized\n");

    return 0;
}

void lds01_start(lds01_t *lds01)
{
    if (lds01->running) {
        return;
    }
    lds01_reset_data(lds01);
    lds01->running = true;
    uart_write(lds01->params.uart, (uint8_t *)"b", 1);
}

void lds01_stop(lds01_t *lds01)
{
    if (!lds01->running) {
        return;
    }
    uart_write(lds01->params.uart, (uint8_t *)"e", 1);
    lds01->running = false;
}

void lds01_set_distance_filter(lds01_t *lds01, uint16_t new_filter)
{
    lds01->filter = new_filter;
}

void lds01_get_distances(lds01_t *lds01, uint16_t *distances)
{
    mutex_lock(&lds01->data_lock);
    memcpy((void *)distances, (void *)lds01->distances, LDS01_NB_ANGLES * sizeof(uint16_t));
    mutex_unlock(&lds01->data_lock);
}

void lds01_get_intensities(lds01_t *lds01, uint16_t *intensities)
{
    mutex_lock(&lds01->data_lock);
    memcpy((void *)intensities, (void *)lds01->intensities, LDS01_NB_ANGLES * sizeof(uint16_t));
    mutex_unlock(&lds01->data_lock);
}
