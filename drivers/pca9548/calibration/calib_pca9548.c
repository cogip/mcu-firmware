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
    puts("\t'a'\t Run defined callback");
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
    char c[2];

    while (c[0] != 'q') {
        printf("Channel number: %u\n", channel_id);

        c[0] = getchar();

        /* useful only for 0 to 7 keys */
        uint8_t channel_selected = (uint8_t)atoi(c);

        switch(c[0]) {
#ifdef PCA9548_CALIB_CB
            /* Calibration callback */
            case 'a':
                PCA9548_CALIB_CB(dev);
                break;
#endif /* PCA9548_CALIB_CB */
            /* Next channel */
            case 'n':
                channel_id = (channel_id >= pca9548_config[dev].channel_numof - 1) ?
                    pca9548_config[dev].channel_numof - 1 : channel_id + 1;
                break;
            /* Previous channel */
            case 'p':
                channel_id = (channel_id <= 0) ? 0 : channel_id - 1;
                break;
            case '0':
            case '1':
            case '2':
            case '3':
            case '4':
            case '5':
            case '6':
            case '7':
                if (channel_selected < pca9548_config[dev].channel_numof)
                    channel_id = channel_selected;
                break;
            default:
                continue;
        }

        pca9548_set_current_channel(dev, channel_id);

    }

    puts("");

    return ret;

pca9548_calib_servo_cmd_err:
    return ret;
}


void pca9548_calib_init(void)
{
    shell_command_t cmd = { 
        "pcc", "pcc <pca9548_id>",
        pca9548_calib_cmd
    };

    pf_add_shell_command(&pf_shell_commands, &cmd);
}
