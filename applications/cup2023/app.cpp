/* Project includes */
#include "app.hpp"
#include "app_conf.hpp"
#include "app_actuators_shell.hpp"
#include "platform.hpp"

void app_init(void)
{
    /* Init shell commands */
    cogip::app::actuators::shell_init();
}
