// Project includes
#include "app.hpp"
#include "app_conf.hpp"
#include "platform.hpp"

// Init all known fixed obstacles on map
static void _fixed_obstacles_init(void)
{}

void app_init(void)
{
    // Init quadpid controller
    pf_init_quadpid_params(ctrl_quadpid_params);

    // Init servomotors to their default position
    sd21_init(sd21_config_app);

    _fixed_obstacles_init();
}

void app_init_tasks(void)
{}
