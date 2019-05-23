#include <stdio.h>

/* Standard includes */
#include <stdlib.h>

/* RIOT includes */
#include "fmt.h"
#include "xtimer.h"

/* Project includes */
#include "planner.h"
#include "platform.h"
#include "calibration/calib_planner.h"

/* Speed correction calibration usage */
static void pln_calib_print_usage(void)
{
    puts(">>> Entering calibration for planner");

    puts("\t'q'\t Quit calibration");
    puts("\t'n'\t Go to next position");
    puts("\t'p'\t Go to previous position");
    puts("\t's'\t Go back to start position");
    puts("\t'N'\t Select next position");
    puts("\t'P'\t Select previous position");
    puts("\t'a'\t Launch action");
}

/* Speed calibration command */
static int pln_calib_cmd(int argc, char **argv)
{
    (void)argv;
    int ret = 0;
    func_cb_t cb = NULL;
    static uint8_t calib_path_index = 0;

    path_t *path = pf_get_path();
    ctrl_t* ctrl = pf_get_ctrl();

    /* Always print usage first */
    pln_calib_print_usage();

    /* Check arguments */
    if (argc > 1) {
        puts("Bad arguments number !");
        ret = -1;
        goto pln_calib_cmd_err;
    }

    /* Key pressed */
    char c = 0;

    pln_set_allow_change_path_pose(FALSE);

    while (c != 'q') {
        /* Display current position index in path */
        printf("Position index: %u\n", calib_path_index);

        /* Wait for a key pressed */
        c = getchar();

        switch(c) {
            /* Next position */
            case 'n':
                if (calib_path_index < path->nb_pose - 1)
                    calib_path_index++;
                path_set_current_pose_idx(path, calib_path_index);
                break;
            case 'p':
                if (calib_path_index > 0)
                    calib_path_index--;
                path_set_current_pose_idx(path, calib_path_index);
                break;
            case 's':
                path_reset_current_pose_idx(path);
                break;
            case 'N':
                if (calib_path_index < path->nb_pose - 1)
                    calib_path_index++;
                break;
            case 'P':
                if (calib_path_index > 0)
                    calib_path_index--;
                break;
            case 'a':
                cb = path_get_pose_at_idx(path, calib_path_index)->act;
                if(cb != NULL) {
                    puts("Launch callback !");
                    (*cb)();
                }
                break;
            default:
                continue;
        }

        pln_start(ctrl);

        /* Data stop signal */
        puts("<<<< STOP >>>>");

        /* Always remind usage */
        pln_calib_print_usage();
    }

pln_calib_cmd_err:
    return ret;
}

/* Init calibration commands */
void pln_calib_init(void)
{
    /* Add planner calibration command */
    shell_command_t cmd_calib_pln = {
        "pc", "Planner calibration",
        pln_calib_cmd
    };

    pf_add_shell_command(&cmd_calib_pln);
}
