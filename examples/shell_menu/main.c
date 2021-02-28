#include <stdio.h>

#include "shell.h"
#include "shell_menu.h"

static int cmd_global(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    puts("Execute global command");

    return 0;
}

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

    menu_init(&menu, "Sub-menu 1");

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


    menu_init(&menu, "Sub-menu 2");

    menu_add_list(&menu, commands);

    menu_enter(&menu);

    return 0;
}

int main(void)
{
    puts("\n== Shell menu example ==");

    const shell_command_t global_commands[] = {
        {"global", "Global command", cmd_global},
        menu_cmd_null
    };
    menu_set_global_commands(global_commands);

    /* Create the main menu */
    shell_menu_t main_menu;

    menu_init(&main_menu, "Main menu");

    const shell_command_t main_menu_commands[] = {
        { "cmd_1", "Command 1", cmd_1 },
        { "cmd_2", "Command 2", cmd_2 },
        { "sub1", "Enter sub-menu 1", cmd_sub_1 },
        { "sub2", "Enter sub-menu 1", cmd_sub_2 },
        menu_cmd_null
    };

    menu_add_list(&main_menu, main_menu_commands);

    /* Push main menu */
    menu_enter(&main_menu);

    /* Start shell */
    menu_start();

    return 0;
}
