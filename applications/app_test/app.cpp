/* Project includes */
#include "app.hpp"
#include "app_conf.hpp"
#include "platform.hpp"

#define ENABLE_DEBUG        (0)
#include "debug.h"

void app_init(void)
{
    /* Init quadpid controller */
    pf_init_quadpid_params(ctrl_quadpid_params);

    cogip::app::app_obstacles_init();
    cogip::app::app_samples_init();
    cogip::app::app_shell_init();
}

void app_init_tasks(void)
{}
