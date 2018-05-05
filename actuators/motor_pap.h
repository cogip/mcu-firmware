#ifndef MOTOR_PAP_H_

#include <stdint.h>

#include "utils.h"

void motor_pap_init(void);
uint8_t motor_pap_turn_next_storage(void);
void motor_pap_unit_test(void);

#endif /* MOTOR_PAP_H_ */
