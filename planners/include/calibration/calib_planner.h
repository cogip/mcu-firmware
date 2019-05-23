#ifndef CALIB_PLN_H_
#define CALIB_PLN_H_

#include "ctrl/quadpid.h"

/**
 * @brief Initialize quadpid calibration, mainly adds commands to RIOT shell.
 *
 * @return
 */
void pln_calib_init(void);

#endif /* CALIB_PLN_H_ */
