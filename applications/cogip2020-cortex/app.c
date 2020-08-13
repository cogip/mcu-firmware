/* System includes */
#include <stdarg.h>
#include <stdio.h>
#include <thread.h>

/* RIOT includes */
#define ENABLE_DEBUG        (0)
#include "debug.h"
#include <motor_driver.h>
#include <periph/qdec.h>
#include "xtimer.h"

/* Project includes */
#include "ctrl/quadpid.h"
#include "obstacle.h"
#include "pca9548.h"
#include "planner.h"
#include "app.h"
#include "app_conf.h"
#include "platform.h"

int app_is_game_launched(void)
{
    /* Starter switch */
    return !gpio_read(GPIO_STARTER);
}

/* Init all known fixed obstacles on map */
static void app_fixed_obstacles_init(void)
{
}


void app_init(void)
{
    /* Init quadpid controller */
    pf_init_quadpid_params(ctrl_quadpid_params);

    app_fixed_obstacles_init();

    const uint8_t camp_left = pf_is_camp_left();
    printf("%s camp\n", camp_left ? "LEFT" : "RIGHT");
}

void app_init_tasks(void)
{
}
