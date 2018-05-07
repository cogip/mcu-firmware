#ifndef MOTOR_PAP_H_

#include <stdint.h>

#include "utils.h"

void motor_pap_init(void);
uint8_t motor_pap_turn_next_storage(void);

#if defined(MODULE_CALIBRATION)
void motor_pap_calib(void);
#endif

#endif /* MOTOR_PAP_H_ */
