/* Project includes */
#include "app.hpp"
#include "app_arm.hpp"
#include "app_conf.hpp"
#include "platform.hpp"

#define ENABLE_DEBUG        (0)
#include "debug.h"

void app_init(void)
{
    // Init quadpid controller
    pf_init_quadpid_params(ctrl_quadpid_params);

    // Starter
    gpio_init(GPIO_STARTER, GPIO_IN);

    // Init all LX servomotors
    cogip::app::app_arms_init();

    // Fixed obstacles
    cogip::app::app_obstacles_init();

    // Samples to take
    cogip::app::app_samples_init();

    // Shell
    cogip::app::app_shell_init();
}

void app_init_tasks(void)
{}
