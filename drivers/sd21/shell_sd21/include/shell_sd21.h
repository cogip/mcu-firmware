#pragma once

#include "sd21.h"

/**
 * @brief SD21 duty cycle step
 */
#define SD21_SERVO_POS_STEP    25

/**
 * @brief Initialize SD21 calibration, mainly adds commands to RIOT shell.
 *
 * @param[in]   sd21_config_new     sd21 input configuration
 *
 * @return
 */
void sd21_shell_init(const sd21_conf_t* sd21_config_new);
