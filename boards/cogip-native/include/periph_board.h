#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#ifndef DOXYGEN
/**
 * @brief    Override I2C clock speed values
 *
 * This is required here to have i2c_speed_t defined in this file.
 * @{
 */
#define HAVE_I2C_SPEED_T
typedef enum {
    I2C_SPEED_LOW = 0,      /**< low speed mode:     ~10 kbit/s */
    I2C_SPEED_NORMAL,       /**< normal mode:       ~100 kbit/s */
    I2C_SPEED_FAST,         /**< fast mode:         ~400 kbit/s */
    I2C_SPEED_FAST_PLUS,    /**< fast plus mode:   ~1000 kbit/s */
    I2C_SPEED_HIGH,         /**< high speed mode:  ~3400 kbit/s */
} i2c_speed_t;
/** @} */
#endif /* ndef DOXYGEN */

/**
 * @brief   I2C configuration structure type
 */
typedef struct {
    void *dummy;
} i2c_conf_t;

/** Use read reg function from periph common */
#define PERIPH_I2C_NEED_READ_REG
#define PERIPH_I2C_NEED_READ_REGS
/** Use write reg function from periph common */
#define PERIPH_I2C_NEED_WRITE_REG
#define PERIPH_I2C_NEED_WRITE_REGS

#ifdef __cplusplus
}
#endif

/** @} */
