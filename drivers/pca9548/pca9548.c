#include <stdio.h>
#include "pca9548.h"

#ifdef MODULE_SHELL_PCA9548
#   include "shell_pca9548.h"
#endif

static uint8_t pca9548_current_channel[PCA9548_NUMOF];

void pca9548_set_current_channel(pca9548_t dev, uint8_t channel)
{
    assert(dev < PCA9548_NUMOF);

    const pca9548_conf_t *pca9548 = &pca9548_config[dev];

    assert(channel < pca9548->channel_numof);

    i2c_acquire(pca9548->i2c_dev_id);

    int err = 1;

    err = i2c_write_byte(pca9548->i2c_dev_id, pca9548->i2c_address, 1 << channel, 0);

    i2c_release(pca9548->i2c_dev_id);

    if (!err) {
        pca9548_current_channel[dev] = channel;
    }
}

uint8_t pca9548_get_current_channel(pca9548_t dev)
{
    assert(dev < PCA9548_NUMOF);

    return pca9548_current_channel[dev];
}

void pca9548_init(void)
{
    for (pca9548_t dev = 0; dev < PCA9548_NUMOF; dev++) {
        const pca9548_conf_t *pca9548 = &pca9548_config[dev];

        assert(pca9548->channel_numof <= PCA9548_CHANNEL_MAX);

        pca9548_set_current_channel(dev, 0);
    }

#ifdef MODULE_SHELL_PCA9548
    pca9548_shell_init();
#endif
}
