#include "lds01.h"
#include "lds01_params.h"

#define ENABLE_DEBUG (0)
#include "debug.h"

#include <string.h>

#include "shmem.h"

const shmem_data_t *shared_data = NULL;

#define LDS01_NUMOF ARRAY_SIZE(lds01_params)    /**< number of used lds01 devices */

/**
 * @brief   LDS01 native descriptor
 */
typedef struct {
    lds01_params_t params;                      /**< parameters */
    bool running;                               /**< set to true if lds01 is running */
    uint16_t filter;                            /**< distance filter in millimeters */
    uint16_t distances[LDS01_NB_ANGLES];        /**< distance data */
    uint16_t intensities[LDS01_NB_ANGLES];      /**< intensity data */
} lds01_dev_t;

/* Allocate memory for the device descriptor */
lds01_dev_t lds01_devs[LDS01_NUMOF];

static uint16_t lds01_get_filtered_distance(lds01_dev_t *lds01_dev, uint16_t distance, uint16_t intensity)
{
    if (lds01_dev->filter == 0) {
        return distance;
    }
    if (intensity == 0 || distance == 0 || distance > lds01_dev->filter) {
        return lds01_dev->filter;
    }
    return distance;
}

void lds01_update_last_frame(const lds01_t lds01)
{
    assert(lds01 < LDS01_NUMOF);
}

int lds01_init(const lds01_t lds01, const lds01_params_t *params)
{
    assert(lds01 < LDS01_NUMOF);

    lds01_dev_t *lds01_dev = &lds01_devs[lds01];

    lds01_dev->params = *params;
    lds01_dev->filter = 0;
    lds01_dev->running = false;

    for (unsigned int i = 0; i < LDS01_NB_ANGLES; i++) {
        lds01_dev->distances[i] = 0;
        lds01_dev->intensities[i] = 1000;
    }

    DEBUG("LDS01 native initialized\n");

    return 0;
}

void lds01_start(const lds01_t lds01)
{
    assert(lds01 < LDS01_NUMOF);

    lds01_dev_t *lds01_dev = &lds01_devs[lds01];

    if (lds01_dev->running) {
        return;
    }
    lds01_dev->running = true;
}

void lds01_stop(const lds01_t lds01)
{
    assert(lds01 < LDS01_NUMOF);

    lds01_dev_t *lds01_dev = &lds01_devs[lds01];

    lds01_dev->running = false;
}

void lds01_set_distance_filter(const lds01_t lds01, uint16_t new_filter)
{
    assert(lds01 < LDS01_NUMOF);

    lds01_dev_t *lds01_dev = &lds01_devs[lds01];

    lds01_dev->filter = new_filter;
}

void lds01_get_distances(const lds01_t lds01, uint16_t *distances)
{
    assert(lds01 < LDS01_NUMOF);

    lds01_dev_t *lds01_dev = &lds01_devs[lds01];

    if (shared_data == NULL) {
        shared_data = shmem_get_data();
    }

    if (shared_data) {
        for (unsigned int i = 0; i < LDS01_NB_ANGLES; i++) {
            uint16_t distance = shared_data->lidar_distances[i];
            if (lds01_dev->params.invert_data) {
                lds01_dev->distances[(LDS01_NB_ANGLES - i) % LDS01_NB_ANGLES] = lds01_get_filtered_distance(lds01_dev, distance, 1);
            }
            else {
                lds01_dev->distances[i] = lds01_get_filtered_distance(lds01_dev, distance, 1);
            }
        }
    }

    memcpy((void *)distances, (void *)lds01_dev->distances, LDS01_NB_ANGLES * sizeof(uint16_t));
}

void lds01_get_intensities(const lds01_t lds01, uint16_t *intensities)
{
    assert(lds01 < LDS01_NUMOF);

    lds01_dev_t *lds01_dev = &lds01_devs[lds01];

    memcpy((void *)intensities, (void *)lds01_dev->intensities, LDS01_NB_ANGLES * sizeof(uint16_t));
}
