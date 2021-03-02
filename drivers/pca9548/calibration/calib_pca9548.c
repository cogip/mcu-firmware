#include <stdio.h>

/* Standard includes */
#include <stdlib.h>

/* RIOT includes */
#include "fmt.h"

/* Project includes */
#include "board.h"
#include "pca9548.h"
#include "platform.h"
#include "calibration/calib_pca9548.h"
#include "calib_platform.h"

/* Shell command array */
static shell_command_linked_t pca9548_shell_commands;
static const char *pca9548_name = "pca9548";

/* Board and servo ids */
static pca9548_t dev = 0;
static uint8_t channel_id = 0;

#ifdef PCA9548_CALIB_CB
static int pca9548_cmd_defined_callback_cb(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    PCA9548_CALIB_CB(dev);

    return EXIT_SUCCESS;
}
#endif /* PCA9548_CALIB_CB */

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

static int pca9548_calib_cmd(int argc, char **argv)
{
    (void)argv;
    int ret = 0;

    /* Check arguments */
    if (argc != 2) {
        puts("Bad number of arguments!");
        ret = -1;
        goto pca9548_calib_servo_cmd_err;
    }

    /* Get board and servo ids */
    dev = atoi(argv[1]);
    channel_id = pca9548_get_current_channel(dev);

    pf_init_shell_commands(&pca9548_shell_commands, pca9548_name);

#ifdef PCA9548_CALIB_CB
    shell_command_t pca9548_cmd_defined_callback = {
        "a", "Run defined callback",
        pca9548_cmd_defined_callback_cb
    };
    pf_add_shell_command(&pca9548_shell_commands, &pln_cmd_defined_callback);
#endif /* PCA9548_CALIB_CB */

    shell_command_t pca9548_cmd_next_channel = {
        "n", "Next channel",
        pca9548_cmd_next_channel_cb
    };
    pf_add_shell_command(&pca9548_shell_commands, &pca9548_cmd_next_channel);

    shell_command_t pca9548_cmd_previous_channel = {
        "p", "Previous channel",
        pca9548_cmd_previous_channel_cb
    };
    pf_add_shell_command(&pca9548_shell_commands, &pca9548_cmd_previous_channel);

    shell_command_t pca9548_cmd_refresh = {
        "r", "Refresh",
        pca9548_cmd_refresh_cb
    };
    pf_add_shell_command(&pca9548_shell_commands, &pca9548_cmd_refresh);

    shell_command_t pca9548_cmd_switch = {
        "switch", "Switch to channel <n> (n between 0 and 9)",
        pca9548_cmd_switch_cb
    };
    pf_add_shell_command(&pca9548_shell_commands, &pca9548_cmd_switch);

    pf_add_shell_command(&pca9548_shell_commands, &cmd_exit_shell);

    /* Push new menu */
    pf_push_shell_commands(&pca9548_shell_commands);

pca9548_calib_servo_cmd_err:
    return ret;
}


void pca9548_calib_init(void)
{
    shell_command_t cmd = { 
        "pcc", "pca9548 calibration <id>",
        pca9548_calib_cmd
    };
    pf_add_shell_command(&pf_shell_commands, &cmd);
}
