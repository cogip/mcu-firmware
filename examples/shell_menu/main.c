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

static int cmd_1_1(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    puts("Execute command 1 in Module 1");
    return 0;
}

static int cmd_1_2(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    puts("Execute command 2 in Module 1");
    return 0;
}

static int cmd_2_1(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    puts("Execute command 1 in Module 2");
    return 0;
}

static int cmd_2_2(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    puts("Execute command 2 in Module 2");
    return 0;
}

static int cmd_2_1_sub(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    puts("Execute command in Module 2 sub-menu");
    return 0;
}

void module1_enter_callback(void)
{
    puts("Execute module1 enter callback");
}

void module1_init(void) {
    shell_menu_t menu = menu_init("Module 1 menu", "mod1", menu_root, module1_enter_callback);
    static const shell_command_t menu_commands[] = {
        { "cmd_1", "Module 1 command 1", cmd_1_1 },
        { "cmd_2", "Module 1 command 2", cmd_1_2 },
        MENU_NULL_CMD
    };
    menu_add_list(menu, menu_commands);
}

void module2_init(void) {
    shell_menu_t menu = menu_init("Module 2 menu", "mod2", menu_root, NULL);
    static const shell_command_t menu_commands[] = {
        { "cmd_1", "Module 2 command 1", cmd_2_1 },
        { "cmd_2", "Module 2 command 2", cmd_2_2 },
        MENU_NULL_CMD
    };
    menu_add_list(menu, menu_commands);

    /* Add a sub menu in the module */
    shell_menu_t sub_menu = menu_init("Module 2 sub-menu", "mod2_sub", menu, NULL);
    static const shell_command_t sub_menu_commands[] = {
        { "cmd_1", "Module 2 sub-menu command", cmd_2_1_sub },
        MENU_NULL_CMD
    };
    menu_add_list(sub_menu, sub_menu_commands);
}

void app_init(void) {
    static shell_command_t global_commands[] = {
        {"global", "Global command", cmd_global},
        MENU_NULL_CMD
    };
    menu_set_global_commands(global_commands);

    const shell_command_t main_menu_commands[] = {
        { "cmd_1", "Command 1", cmd_1 },
        { "cmd_2", "Command 2", cmd_2 },
        MENU_NULL_CMD
    };
    menu_add_list(menu_root, main_menu_commands);
}

int main(void)
{
    puts("\n== Shell menu example ==");

    app_init();
    module1_init();
    module2_init();

    /* Start shell */
    menu_start();

    return 0;
}
