/* Project includes */
#include "app.hpp"
#include "app_conf.hpp"
#include "app_arm_shell.hpp"
#include "platform.hpp"

void app_init(void)
{
    /* Init quadpid controller */
    pf_init_quadpid_params(ctrl_quadpid_params);

    /* Init shell commands */
    cogip::app::arms::shell_init();
}
