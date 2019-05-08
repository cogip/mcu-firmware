#ifndef CALIB_QUADPID_H_
#define CALIB_QUADPID_H_

#include "ctrl/quadpid.h"

/**
 * @brief Initialize quadpid calibration, mainly adds commands to RIOT shell.
 *
 * @return
 */
void ctrl_quadpid_calib_init(void);

#endif /* CALIB_QUADPID_H_ */
