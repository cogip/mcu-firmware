/* Project includes */
#include "app.hpp"
#include "app_conf.hpp"
#include "platform.hpp"

#define ENABLE_DEBUG        (0)
#include "debug.h"

/* Init all known fixed obstacles on map */
static void _fixed_obstacles_init(void)
{}

void app_init(void)
{
    /* Init quadpid controller */
    pf_init_quadpid_params(ctrl_quadpid_params);

    _fixed_obstacles_init();
}

void app_init_tasks(void)
{}
