#include "app_samples.hpp"
#include "app_wizard.hpp"

#include "platform.hpp"
#include "shell_menu/shell_menu.hpp"

namespace cogip {

namespace app {

static int _cmd_wizard(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    app_wizard();
    return 0;
}

static int _cmd_samples(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    cogip::app::app_samples_request();
    return 0;
}

void app_shell_init()
{
    cogip::shell::root_menu.push_back(
        new cogip::shell::Command("_wizard", "Run Wizard", _cmd_wizard));

    cogip::shell::root_menu.push_back(
        new cogip::shell::Command("_samples", "Get detected samples", _cmd_samples));
}

} // namespace app

} // namespace cogip
