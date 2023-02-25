
#include "shell_menu/shell_menu.hpp"

namespace cogip {
namespace app {
namespace arms {

{
    (void)argc;
    (void)argv;
    return EXIT_SUCCESS;
}

static void _shell_enter_callback(void)
{
}

static cogip::shell::Menu arm_menu = { "Arm actions", "arm", &cogip::shell::root_menu(), _shell_enter_callback };

void shell_init()
{
}

} // namespace arms
} // namespace app
} // namespace cogip
