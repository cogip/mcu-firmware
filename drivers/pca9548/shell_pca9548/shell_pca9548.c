#include <stdio.h>

/* Standard includes */
#include <stdlib.h>

/* RIOT includes */
#include "fmt.h"

/* Project includes */
#include "board.h"
#include "pca9548.h"
#include "platform.h"
#include "shell_menu.h"

/* Board and servo ids */
static pca9548_t dev = 0;
static uint8_t channel_id = 0;

static int pca9548_cmd_defined_callback_cb(int argc, char **argv)
{
    (void)argc;
    (void)argv;

#ifdef PCA9548_SHELL_CB
    PCA9548_SHELL_CB(dev);
#else
    puts("No callback defined !");
#endif /* PCA9548_SHELL_CB */

    return EXIT_SUCCESS;
}

static int pca9548_cmd_next_channel_cb(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    channel_id = (channel_id >= pca9548_config[dev].channel_numof - 1) ?
        pca9548_config[dev].channel_numof - 1 : channel_id + 1;

    return EXIT_SUCCESS;
}

static int pca9548_cmd_previous_channel_cb(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    channel_id = (channel_id <= 0) ? 0 : channel_id - 1;

    return EXIT_SUCCESS;
}

static int pca9548_cmd_refresh_cb(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    channel_id = (channel_id <= 0) ? 0 : channel_id - 1;

    return EXIT_SUCCESS;
}

static int pca9548_cmd_switch_cb(int argc, char **argv)
{
    /* Check arguments */
    if (argc != 2) {
        puts("Bad number of arguments!");
        return EXIT_FAILURE;
    }

    uint8_t channel_selected = (uint8_t)atoi(argv[1]);

    switch (channel_selected)
    {
        case 0:
        case 1:
        case 2:
        case 3:
        case 4:
        case 5:
        case 6:
        case 7:
        case 8:
        case 9:
            if (channel_selected < pca9548_config[dev].channel_numof) {
                channel_id = channel_selected;
            }
            break;
        default:
            break;
    }

    return EXIT_SUCCESS;
}

void pca9548_shell_init(void)
{
    /* pca9548 menu and commands */
    shell_menu_t menu = menu_init("PCA9548 menu", "pca9548_menu", menu_root);

    const shell_command_t shell_pca9548_menu_commands[] = {
        { "a", "Run defined callback", pca9548_cmd_defined_callback_cb },
        { "n", "Next channel", pca9548_cmd_next_channel_cb },
        { "p", "Previous channel", pca9548_cmd_previous_channel_cb },
        { "r", "Refresh", pca9548_cmd_refresh_cb },
        { "switch", "Switch to channel <n> (n between 0 and 9)",
            pca9548_cmd_switch_cb },
        MENU_NULL_CMD,
    };

    menu_add_list(menu, shell_pca9548_menu_commands);
}
