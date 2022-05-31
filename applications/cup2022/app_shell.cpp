#include "app_samples.hpp"
#include "app_wizard.hpp"

#include "platform.hpp"
#include "shell_menu/shell_menu.hpp"

#include "thread.h"

static char wizard_thread_stack[THREAD_STACKSIZE_MEDIUM];
#define WIZARD_THREAD_PRIO (THREAD_PRIORITY_MAIN - 1)

namespace cogip {

namespace app {

static void *_wizard_thread_command(void *arg)
{
    (void)arg;
    app_wizard();
    return EXIT_SUCCESS;
}

static int _cmd_wizard(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    thread_create(
        wizard_thread_stack,
        sizeof(wizard_thread_stack),
        WIZARD_THREAD_PRIO,
        0,
        _wizard_thread_command,
        NULL,
        "Wizard"
        );

    return 0;
}

static int _cmd_samples(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    cogip::app::app_samples_detect();
    return 0;
}

void app_shell_init()
{
    cogip::shell::root_menu.push_back(
        new cogip::shell::Command("wizard", "Run Wizard", _cmd_wizard));

    cogip::shell::root_menu.push_back(
        new cogip::shell::Command("_samples", "Get detected samples", _cmd_samples));
}

} // namespace app

} // namespace cogip
