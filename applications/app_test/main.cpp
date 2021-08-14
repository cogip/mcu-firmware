#include <stdio.h>

#include "app.h"
#include "platform.h"
#include "shell_menu.h"

int main(void)
{
    pf_init();
    app_init();

    pf_init_tasks();
    app_init_tasks();

    /* Start shell */
    menu_start();

    return 0;
}
