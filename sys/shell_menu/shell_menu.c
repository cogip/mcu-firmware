#include <assert.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "shell_menu.h"

const shell_command_t menu_cmd_null = {NULL, NULL, NULL};

static shell_menu_t current_menu;

static shell_menu_t main_menu;

static int _display_json_help(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    printf("{\"name\": \"%s\", \"entries\": [", current_menu.name);
    shell_command_t *cmd = (shell_command_t*)&current_menu;
    for (; cmd->name != NULL; cmd++) {
        if(cmd != (shell_command_t*)&current_menu) {
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

void menu_init_menu(shell_menu_t *menu, const char *name) {
    menu->name = name;
    menu->current = menu;
    menu->previous = NULL;

    for(uint8_t i = 0 ; i < NB_SHELL_COMMANDS ; i++) {
        menu->shell_commands[i] = menu_cmd_null;
    }

    menu_add_one(menu, &cmd_exit_menu);
    menu_add_one(menu, &cmd_help_json);
}

void menu_add_one(shell_menu_t *menu, const shell_command_t *command)
{
    uint8_t command_id = 0;

    for (shell_command_t *cmd = menu->shell_commands ; cmd->name != NULL; cmd++, command_id++) {}

    assert(command_id < NB_SHELL_COMMANDS);

    menu->shell_commands[command_id++] = *command;
}

void menu_add_list(shell_menu_t *menu, const shell_command_t commands[])
{
    for (const shell_command_t *cmd = commands; cmd->name != NULL; cmd++) {
        menu_add_one(menu, cmd);
    }
}

void menu_enter(shell_menu_t *menu) {
    menu->previous = current_menu.current;
    memcpy(&current_menu, menu, sizeof(shell_menu_t));
    printf("Enter shell menu: %s\n", menu->name);
}

void menu_exit(void) {
    if (current_menu.previous) {
        printf("Exit shell menu: %s\n", current_menu.name);
        memcpy(&current_menu, current_menu.previous, sizeof(shell_menu_t));
        printf("Enter shell menu: %s\n", current_menu.name);
    }
}

void menu_start(void)
{
    char line_buf[SHELL_DEFAULT_BUFSIZE];

    /* Push main menu */
    menu_enter(&main_menu);

    shell_run((shell_command_t*)&main_menu, line_buf, SHELL_DEFAULT_BUFSIZE);
}

shell_menu_t *menu_get_main_menu(void)
{
    return &main_menu;
}

void menu_init(const char *name) {
    /* Create the main menu */
    menu_init_menu(&main_menu, name);
}
