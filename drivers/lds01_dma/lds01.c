#include "lds01.h"
#include "lds01_params.h"

#define ENABLE_DEBUG (0)
#include "debug.h"

#include <string.h>

#include "cpu.h"
#include "periph/uart.h"
#include "periph/gpio.h"
#include "stm32f446xx.h"

#define LDS01_NUMOF ARRAY_SIZE(lds01_params)    /**< number of used lds01 devices */
#define LDS01_FRAME_SYNC_BYTE (0xFA)            /**< first byte of a frame */
#define LDS01_NB_FRAMES (40U)                   /**< number of frames to get all angles */
#define LDS01_NB_ANGLES_BY_FRAME (6U)           /**< number of angles in one frame */

#define USART3_RX_DMA_STREAM DMA1_Stream1
#define USART3_RX_DMA_CHAN 4

#define LDS01_UART_BAUD (230400U)                   /**< lds01 UART baud rate */
#define LDS01_FRAME_SIZE (sizeof(lds01_frame_t))    /**< lds01 frame size */
#define LDS01_BUFFER_SIZE (LDS01_FRAME_SIZE * 4)    /**< size of the ringbuffer used to store raw data from UART */

/**
 * @brief   LDS01 offset
 */
typedef struct {
    uint8_t intensity[2];   /**< intentity bytes */
    uint8_t distance[2];    /**< distance bytes */
    // cppcheck-suppress unusedStructMember
    uint8_t reserved[2];    /**< reserved bytes */
} lds01_offset_t;

/**
 * @brief   LDS01 frame
 */
typedef struct {
    uint8_t sync;               /**< begin frame byte (0xAF) */
    uint8_t index;              /**< frame index */
    uint8_t rpm[2];             /**< rpm (unused)*/
    lds01_offset_t offsets[6];  /**< frame offsets */
    uint8_t checksum[2];        /**< frame checksum */
} lds01_frame_t;

/**
 * @brief   LDS01 descriptor
 */
typedef struct {
    lds01_params_t params;                      /**< parameters */
    bool running;                               /**< set to true if lds01 is running */
    uint16_t filter;                            /**< distance filter in millimeters */
    uint16_t min_intensity;                     /**< minimum intensity to consider the data valid */
    mutex_t data_lock;                          /**< lock protecting data access */
    uint16_t distances[LDS01_NB_ANGLES];        /**< distance data */
    uint16_t intensities[LDS01_NB_ANGLES];      /**< intensity data */
    char rx_mem[LDS01_BUFFER_SIZE];             /**< raw frame buffer */
    ringbuffer_t rx_buf;                        /**< frame ringbuffer */
    uint8_t dma_rx_buf[LDS01_FRAME_SIZE * 2];   /**< DMA read buffer */
} lds01_dev_t;

/* Allocate memory for the device descriptor */
lds01_dev_t lds01_devs[LDS01_NUMOF];

void isr_dma1_stream1(void)
{
    lds01_dev_t *lds01_dev = &lds01_devs[0];
    uint8_t *buf_ptr = NULL;

    /* Handle half transfer interrupt */
    if (DMA1->LISR & DMA_LISR_HTIF1_Msk) {
        buf_ptr = lds01_dev->dma_rx_buf;
    }

    /* Handle transfer complete interrupt */
    if (DMA1->LISR & DMA_LISR_TCIF1_Msk) {
        buf_ptr = lds01_dev->dma_rx_buf + LDS01_FRAME_SIZE;
    }

    /* Copy transfered data in ringbuffer and send an event to the thread */
    if (buf_ptr) {
        ringbuffer_add(&(lds01_dev->rx_buf), (const char *)buf_ptr, LDS01_FRAME_SIZE);
        if (lds01_dev->params.new_frame_cb) {
            lds01_dev->params.new_frame_cb();
        }
        else {
            ringbuffer_remove(&(lds01_dev->rx_buf), LDS01_FRAME_SIZE);
        }
    }

    DMA1->LIFCR = (
        DMA_LISR_FEIF0 | DMA_LISR_DMEIF0 |
        DMA_LISR_TEIF0 | DMA_LISR_HTIF0  |
        DMA_LISR_TCIF0
        ) << 6;

    cortexm_isr_end();
}

static void lds01_reset_data(lds01_dev_t *lds01_dev)
{
    memset((void *)lds01_dev->dma_rx_buf, 0, 42 * 2);
    memset((void *)lds01_dev->distances, 0, LDS01_NB_ANGLES * sizeof(uint16_t));
    memset((void *)lds01_dev->intensities, 0, LDS01_NB_ANGLES * sizeof(uint16_t));
    ringbuffer_init(&(lds01_dev->rx_buf), lds01_dev->rx_mem, LDS01_BUFFER_SIZE);
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

static uint16_t lds01_get_filtered_distance(lds01_dev_t *lds01_dev, uint16_t distance, uint16_t intensity)
{
    if (lds01_dev->filter == 0) {
        return distance;
    }
    if (intensity < lds01_dev->min_intensity || distance == 0 || distance > lds01_dev->filter) {
        return lds01_dev->filter;
    }
    return distance;
}

void lds01_update_last_frame(const lds01_t lds01)
{
    assert(lds01 < LDS01_NUMOF);

    lds01_dev_t *lds01_dev = &lds01_devs[lds01];

    lds01_frame_t frame;
    uint16_t distances_tmp[LDS01_NB_ANGLES_BY_FRAME];
    uint16_t intensities_tmp[LDS01_NB_ANGLES_BY_FRAME];
    unsigned bytes_read;

    bool skip = false;

    while (!ringbuffer_empty(&(lds01_dev->rx_buf)) &&
           ringbuffer_peek_one(&(lds01_dev->rx_buf)) != LDS01_FRAME_SYNC_BYTE) {
        ringbuffer_get_one(&(lds01_dev->rx_buf));
        skip = true;
    }
    if (skip) {
        return;
    }

    bytes_read = ringbuffer_get(&(lds01_dev->rx_buf), (char *)&frame, LDS01_FRAME_SIZE);

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
        distances_tmp[i] = lds01_get_filtered_distance(lds01_dev, distance, intensity);
        intensities_tmp[i] = intensity;
    }

    uint8_t index = frame.index - 0xA0;

    /* Update values in arrays given in parameters*/
    mutex_lock(&lds01_dev->data_lock);
    memcpy(
        (void *)(&lds01_dev->distances[index * LDS01_NB_ANGLES_BY_FRAME]),
        (void *)distances_tmp, LDS01_NB_ANGLES_BY_FRAME * sizeof(uint16_t));
    memcpy(
        (void *)(&lds01_dev->intensities[index * LDS01_NB_ANGLES_BY_FRAME]),
        (void *)intensities_tmp, LDS01_NB_ANGLES_BY_FRAME * sizeof(uint16_t));
    mutex_unlock(&lds01_dev->data_lock);
}

int lds01_init(const lds01_t lds01, const lds01_params_t *params)
{
    assert(lds01 < LDS01_NUMOF);

    lds01_dev_t *lds01_dev = &lds01_devs[lds01];

    lds01_dev->params = *params;
    lds01_dev->filter = 0;
    lds01_dev->min_intensity = 0;
    lds01_reset_data(lds01_dev);
    mutex_init(&lds01_dev->data_lock);

    int res = uart_init(lds01_dev->params.uart, LDS01_UART_BAUD, NULL, NULL);

    if (res == UART_NOBAUD) {
        printf("Error: Given baudrate (%u) not possible\n", LDS01_UART_BAUD);
        return 1;
    }
    else if (res != UART_OK) {
        puts("Error: Unable to initialize UART device");
        return 1;
    }

    /* Initialize UART RX */
    gpio_init(uart_config[lds01_dev->params.uart].rx_pin, GPIO_IN_PU);
    gpio_init_af(uart_config[lds01_dev->params.uart].rx_pin, \
                 uart_config[lds01_dev->params.uart].rx_af);

    USART3->CR1 |= USART_CR1_RE | USART_CR1_RXNEIE;
    USART3->CR3 |= USART_CR3_DMAR;

    /* Initialize DMA */
    NVIC_EnableIRQ(DMA1_Stream1_IRQn);

    USART3_RX_DMA_STREAM->FCR = 0;                                          /* Disable FIFO mode => enable direct mode */

    USART3_RX_DMA_STREAM->PAR = (uint32_t)&(USART3->DR);                    /* Set periph address to UART data register */
    USART3_RX_DMA_STREAM->M0AR = (uint32_t)lds01_dev->dma_rx_buf;           /* Set memory address to local buffer */
    USART3_RX_DMA_STREAM->NDTR = LDS01_FRAME_SIZE * 2;                      /* Set transfert size (2 frames) */

    USART3_RX_DMA_STREAM->CR = 0;                                           /* Reset control register */
    USART3_RX_DMA_STREAM->CR |= USART3_RX_DMA_CHAN << DMA_SxCR_CHSEL_Pos;   /* Channel selection */
    USART3_RX_DMA_STREAM->CR |= 0 << DMA_SxCR_PSIZE_Pos;                    /* Peripheral data size = 8-bit */
    USART3_RX_DMA_STREAM->CR |= 0 << DMA_SxCR_MSIZE_Pos;                    /* Memory data size = 8-bit */
    USART3_RX_DMA_STREAM->CR |= 0 << DMA_SxCR_PINC_Pos;                     /* Don't auto-increment periph pointer */
    USART3_RX_DMA_STREAM->CR |= 1 << DMA_SxCR_MINC_Pos;                     /* Auto-increment memory pointer */
    USART3_RX_DMA_STREAM->CR |= DMA_PERIPH_TO_MEM << DMA_SxCR_DIR_Pos;      /* Data transfer direction */
    USART3_RX_DMA_STREAM->CR |= DMA_SxCR_CIRC;                              /* Enable circular mode */
    USART3_RX_DMA_STREAM->CR |= DMA_SxCR_TCIE;                              /* Enable complete transfer interruption */
    USART3_RX_DMA_STREAM->CR |= DMA_SxCR_HTIE;                              /* Enable half transfer interruption */
    USART3_RX_DMA_STREAM->CR |= DMA_SxCR_TEIE;                              /* Enable transfer error interrupt */

    lds01_dev->running = true;                                              /* force stop on startup */
    lds01_stop(lds01);

    DEBUG("LDS01 UART initialized\n");

    return 0;
}

void lds01_start(const lds01_t lds01)
{
    assert(lds01 < LDS01_NUMOF);

    lds01_dev_t *lds01_dev = &lds01_devs[lds01];

    if (lds01_dev->running) {
        return;
    }
    lds01_reset_data(lds01_dev);
    lds01_dev->running = true;
    uart_write(lds01_dev->params.uart, (uint8_t *)"b", 1);
    USART3_RX_DMA_STREAM->CR |= DMA_SxCR_EN;
}

void lds01_stop(const lds01_t lds01)
{
    assert(lds01 < LDS01_NUMOF);

    lds01_dev_t *lds01_dev = &lds01_devs[lds01];

    if (!lds01_dev->running) {
        return;
    }
    uart_write(lds01_dev->params.uart, (uint8_t *)"e", 1);
    USART3_RX_DMA_STREAM->CR &= ~DMA_SxCR_EN;
    lds01_dev->running = false;
}

void lds01_set_distance_filter(const lds01_t lds01, uint16_t new_filter)
{
    assert(lds01 < LDS01_NUMOF);

    lds01_dev_t *lds01_dev = &lds01_devs[lds01];

    lds01_dev->filter = new_filter;
}

void lds01_set_min_intensity(const lds01_t lds01, uint16_t new_min_intensity)
{
    assert(lds01 < LDS01_NUMOF);

    lds01_dev_t *lds01_dev = &lds01_devs[lds01];

    lds01_dev->min_intensity = new_min_intensity;
}

void lds01_get_distances(const lds01_t lds01, uint16_t *distances)
{
    assert(lds01 < LDS01_NUMOF);

    lds01_dev_t *lds01_dev = &lds01_devs[lds01];

    mutex_lock(&lds01_dev->data_lock);
    if (lds01_dev->params.invert_data) {
        for (uint16_t i = 0; i < LDS01_NB_ANGLES; i++) {
            distances[i] = lds01_dev->distances[(LDS01_NB_ANGLES - i) % LDS01_NB_ANGLES];
        }
    }
    else {
        memcpy((void *)distances, (void *)lds01_dev->distances, LDS01_NB_ANGLES * sizeof(uint16_t));
    }
    mutex_unlock(&lds01_dev->data_lock);
}

void lds01_get_intensities(const lds01_t lds01, uint16_t *intensities)
{
    assert(lds01 < LDS01_NUMOF);

    lds01_dev_t *lds01_dev = &lds01_devs[lds01];

    mutex_lock(&lds01_dev->data_lock);
    if (lds01_dev->params.invert_data) {
        for (uint16_t i = 0; i < LDS01_NB_ANGLES; i++) {
            intensities[i] = lds01_dev->intensities[(LDS01_NB_ANGLES - i) % LDS01_NB_ANGLES];
        }
    }
    else {
        memcpy((void *)intensities, (void *)lds01_dev->intensities, LDS01_NB_ANGLES * sizeof(uint16_t));
    }
    mutex_unlock(&lds01_dev->data_lock);
}
