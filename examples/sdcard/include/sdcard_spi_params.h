#pragma once

#include "board.h"

#ifdef __cplusplus
extern "C" {
#endif

static const sdcard_spi_params_t sdcard_spi_params[] = {
    {
        .spi_dev = SPI_DEV(0),
        .cs = GPIO_PIN(PORT_A, 8),
        .clk = GPIO_PIN(PORT_B, 3),
        .mosi = GPIO_PIN(PORT_B, 5),
        .miso = GPIO_PIN(PORT_B, 4),
        .power = GPIO_UNDEF,
        .power_act_high = true
    }
};

#ifdef __cplusplus
}
#endif
