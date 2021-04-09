#include <assert.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "shell_menu.h"

#define MENU_MAX_DESC_SIZE 128

const shell_menu_t menu_root = 0;

static shell_menu_data_t shell_menus[NB_SHELL_MENUS];
static shell_menu_data_t current_menu;
static unsigned int nb_menus = 0;
static const shell_command_t *global_commands = NULL;
static char menu_desc[NB_SHELL_MENUS][MENU_MAX_DESC_SIZE];

static shell_menu_t _find_menu_by_command(const char *cmd)
{
    shell_menu_t menu;

    for (menu = 1; menu < nb_menus; menu++) {
        if (strcmp(cmd, shell_menus[menu].cmd) == 0) {
            break;
        }
    }

    return menu;
}

static int _cmd_enter_sub_menu(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    shell_menu_t menu = _find_menu_by_command(argv[0]);

    if (menu < nb_menus) {
        menu_enter(menu);
    }

    return 0;
}


static int _display_json_help(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    printf("{\"name\": \"%s\", \"entries\": [", current_menu.name);
    shell_command_t *cmd = (shell_command_t *)&current_menu;
    for (; cmd->name != NULL; cmd++) {
        if (cmd != (shell_command_t *)&current_menu) {
            printf(", ");
        }
        printf(
            "{\"cmd\": \"%s\", \"desc\": \"%s\"}",
            cmd->name,
            cmd->desc
            );
    }
    printf("]}\n");
    return EXIT_SUCCESS;
}

static int _exit_menu(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    menu_exit();

    return EXIT_SUCCESS;
}

const shell_command_t cmd_exit_menu = {
    "exit", "Exit current menu",
    _exit_menu
};

const shell_command_t cmd_help_json = {
    "_help_json", "Display available commands in JSON format",
    _display_json_help
};

void menu_set_global_commands(const shell_command_t command[])
{
    global_commands = command;
    menu_add_list(menu_root, global_commands);
}

static void _init_main_menu(void)
{
    if (nb_menus != 0) {
        return;
    }

    shell_menus[menu_root].name = "Main menu";
    shell_menus[menu_root].current = &shell_menus[menu_root];
    shell_menus[menu_root].previous = NULL;
    for (uint8_t i = 0; i < NB_SHELL_COMMANDS; i++) {
        shell_menus[menu_root].shell_commands[i] = (const shell_command_t)MENU_NULL_CMD;
    }
    nb_menus++;

    menu_add_one(menu_root, &cmd_exit_menu);
    menu_add_one(menu_root, &cmd_help_json);
}

shell_menu_t menu_init(const char *name, const char *cmd, const shell_menu_t parent, func_cb_t enter_cb)
{
    _init_main_menu();

    assert(nb_menus < NB_SHELL_MENUS);
    assert(parent < nb_menus);

    shell_menu_t menu = _find_menu_by_command(cmd);
    assert(menu == nb_menus);

    menu = nb_menus++;

    shell_menus[menu].name = name;
    shell_menus[menu].cmd = cmd;
    shell_menus[menu].current = &shell_menus[menu];
    shell_menus[menu].previous = NULL;
    shell_menus[menu].enter_cb = enter_cb;

    for (uint8_t i = 0; i < NB_SHELL_COMMANDS; i++) {
        shell_menus[menu].shell_commands[i] = (const shell_command_t)MENU_NULL_CMD;
    }

    menu_add_one(menu, &cmd_exit_menu);
    menu_add_one(menu, &cmd_help_json);

    if (global_commands) {
        menu_add_list(menu, global_commands);
    }

    sprintf(menu_desc[menu], "Enter %s", name);
    const shell_command_t enter_cmd = { cmd, menu_desc[menu], _cmd_enter_sub_menu };
    menu_add_one(parent, &enter_cmd);

    return menu;
}

void menu_add_one(const shell_menu_t menu, const shell_command_t *command)
{
    if (menu == menu_root) {
        _init_main_menu();
    }

    assert(menu < nb_menus);

    uint8_t command_id = 0;

    for (shell_command_t *cmd = shell_menus[menu].shell_commands; cmd->name != NULL; cmd++, command_id++) {}

    assert(command_id < NB_SHELL_COMMANDS);

    shell_menus[menu].shell_commands[command_id++] = *command;
}

void menu_add_list(const shell_menu_t menu, const shell_command_t commands[])
{
    for (const shell_command_t *cmd = commands; cmd->name != NULL; cmd++) {
        menu_add_one(menu, cmd);
    }
}

void menu_enter(const shell_menu_t menu)
{
    assert(menu < nb_menus);

    if (menu != menu_root) {
        shell_menus[menu].previous = current_menu.current;
    }
    memcpy(&current_menu, &shell_menus[menu], sizeof(shell_menu_data_t));
    printf("Enter shell menu: %s\n", current_menu.name);
    if (current_menu.enter_cb) {
        current_menu.enter_cb();
    }
}

void menu_exit(void)
{
    if (current_menu.previous) {
        printf("Exit shell menu: %s\n", current_menu.name);
        memcpy(&current_menu, current_menu.previous, sizeof(shell_menu_data_t));
        printf("Enter shell menu: %s\n", current_menu.name);
    }
}

void menu_start(void)
{
    _init_main_menu();

    char line_buf[SHELL_DEFAULT_BUFSIZE];

    /* Push main menu */
    menu_enter(menu_root);

    shell_run((shell_command_t *)&current_menu, line_buf, SHELL_DEFAULT_BUFSIZE);
}
