#ifndef CALIB_SD21_H_
#define CALIB_SD21_H_

#include "sd21.h"

/**
 * @brief SD21 duty cycle step
 */
#define SD21_SERVO_POS_STEP    25

/**
 * @brief Initialize SD21 calibration, mainly adds commands to RIOT shell.
 *
 * @return
 */
void sd21_calib_init(void);

#endif /* CALIB_SD21_H_ */
