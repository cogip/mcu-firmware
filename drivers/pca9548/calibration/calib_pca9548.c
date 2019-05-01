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

static void pca9548_calib_print_usage(pca9548_t dev)
{
    printf(">>> Entering calibration for pca9548 %u\n\n",
            dev);

#ifdef PCA9548_CALIB_CB
    puts("\t'c'\t Run defined callback");
#endif /* PCA9548_CALIB_CB */
    puts("\t'n'\t Next channel");
    puts("\t'p'\t Previous channel");
    puts("\t'r'\t Refresh");
}

static int pca9548_calib_cmd(int argc, char **argv)
{
    (void)argv;
    int ret = 0;

    /* Check arguments */
    if (argc != 2) {
        puts("Bad arguments number !");
        ret = -1;
        goto pca9548_calib_servo_cmd_err;
    }

    /* Get board and servo ids */
    pca9548_t dev = atoi(argv[1]);
    uint8_t channel_id = pca9548_get_current_channel(dev);

    /* Display usage */
    pca9548_calib_print_usage(dev);

    puts("");
    puts("=======================");
    puts("");

    /* Key pressed */
    char c = 0;

    while (c != 'q') {
        printf("Channel number: %u\n", channel_id);

        c = getchar();

        switch(c) {
#ifdef PCA9548_CALIB_CB
            /* Calibration callback */
            case 'c':
                PCA9548_CALIB_CB(dev);
                break;
#endif /* PCA9548_CALIB_CB */
            /* Next channel */
            case 'n':
                channel_id = (channel_id >= pca9548_config[dev].channel_numof - 1) ?
                    pca9548_config[dev].channel_numof - 1 : channel_id + 1;
                break;
            default:
                continue;
        }

    }

    puts("");

    return ret;

pca9548_calib_servo_cmd_err:
    return ret;
}


void pca9548_calib_init(void)
{
    shell_command_t cmd = { 
        "pca9548_calib", "pca9548_calib <pca9548_id>",
        pca9548_calib_cmd
    };

    pf_add_shell_command(&cmd);
}
