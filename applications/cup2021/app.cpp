// Project includes
#include "app.hpp"
#include "app_conf.hpp"
#include "platform.hpp"

void app_init(void)
{
    // Init quadpid controller
    pf_init_quadpid_params(ctrl_quadpid_params);

    // Init servomotors to their default position
    sd21_init(sd21_config_app);
}

void app_init_tasks(void)
{}
