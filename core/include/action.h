#ifndef ACTION_H_

//typedef struct {
//	pose_t pos;
//	action_t action;
//} action_t;

#include "utils.h"

typedef func_cb_t action_t;

/* catch lunar modules */
void act_catch_module_front_right(void);
void act_catch_module_rear_right(void);
void act_catch_module_front_left(void);
void act_catch_module_rear_left(void);

#endif /* ACTION_H_ */
