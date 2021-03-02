#include <stdio.h>

#include "shell.h"
#include "shell_menu.h"

static int cmd_1_1(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    puts("Execute command 1 in sub-menu 1");

    return 0;
}

static int cmd_1_2(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    puts("Execute command 2 in sub-menu 1");

    return 0;
}

static int cmd_2_1(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    puts("Execute command 1 in sub-menu 2");

    return 0;
}

static int cmd_2_2(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    puts("Execute command 2 in sub-menu 2");

    return 0;
}

static int cmd_1(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    puts("Execute command 1 in main menu");

    return 0;
}

static int cmd_2(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    puts("Execute command 2 in main menu");

    return 0;
}

static int cmd_sub_1(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    shell_menu_t menu;
    const shell_command_t commands[] = {
        { "cmd_1_1", "Command 1/1", cmd_1_1 },
        { "cmd_1_2", "Command 1/2", cmd_1_2 },
        { NULL, NULL, NULL }
    };

    menu_init_menu(&menu, "Sub-menu 1");

    menu_add_list(&menu, commands);

    menu_enter(&menu);

    return 0;
}

static int cmd_sub_2(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    shell_menu_t menu;
    const shell_command_t commands[] = {
        { "cmd_2_1", "Command 2/1", cmd_2_1 },
        { "cmd_2_2", "Command 2/2", cmd_2_2 },
        { NULL, NULL, NULL }
    };

    menu_init_menu(&menu, "Sub-menu 2");

    menu_add_list(&menu, commands);

    menu_enter(&menu);

    return 0;
}

int main(void)
{
    puts("\n== Shell menu example ==");

    menu_init("Main menu");

    shell_menu_t * const main_menu = menu_get_main_menu();

    const shell_command_t main_menu_commands[] = {
        { "cmd_1", "Command 1", cmd_1 },
        { "cmd_2", "Command 2", cmd_2 },
        { "sub1", "Enter sub-menu 1", cmd_sub_1 },
        { "sub2", "Enter sub-menu 1", cmd_sub_2 },
        menu_cmd_null
    };

    menu_add_list(main_menu, main_menu_commands);

    /* Start shell */
    menu_start();

    return 0;
}
