#ifndef ACTION_H_

#include "utils.h"

/* Actions functions must be of type "func_cb_t" */

void action_init(void); // call at boot time once

void act_catch_same_color_water(void);
void act_launch_same_color_water(void);
void act_catch_interleaved_water(void);
void act_launch_interleaved_water(void);
void act_drop_recycled_water(void);
void act_open_bee_pusher(void);
void act_close_bee_pusher(void);

#endif /* ACTION_H_ */
