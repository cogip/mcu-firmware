#include "app.hpp"
#include "platform.hpp"
#include "shell_menu/shell_menu.hpp"

int main(void)
{
    pf_init();
    app_init();

    pf_init_tasks();
    app_init_tasks();

    // Start shell
    cogip::shell::start();

    return 0;
}
