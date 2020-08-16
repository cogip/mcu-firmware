#include <stdio.h>

/* Standard includes */
#include <stdlib.h>

/* RIOT includes */
#include "fmt.h"

/* Project includes */
#include "platform.h"
#include "sd21.h"
#include "calibration/calib_sd21.h"

static const sd21_conf_t* sd21_config = NULL;
static size_t sd21_numof = 0;

static void sd21_calib_print_usage(sd21_t dev, uint8_t servo_id)
{
    printf(">>> Entering calibration for servo %u of SD21 board %u\n\n",
            servo_id, dev);

    puts("\t'o'\t Opened position");
    puts("\t'c'\t Closed position");
    puts("\t'n'\t Next servomotor");
    puts("\t'p'\t Previous servomotor");
    puts("\t'q'\t Quit calibration");
    puts("\t'r'\t Reset to center position");
    printf("\t'+'\t Add %u microseconds to current position\n",
            SD21_SERVO_POS_STEP);
    printf("\t'-'\t Substract %u microseconds to current position\n",
            SD21_SERVO_POS_STEP);
    puts("\t'0..1'\t Switch to predefined positions (between 0 and 9)");
}

static int sd21_calib_servo_cmd(int argc, char **argv)
{
    (void)argv;
    int ret = 0;

    /* Check arguments */
    if (argc != 3) {
        puts("Bad arguments number !");
        ret = -1;
        goto sd21_calib_servo_cmd_err;
    }

    /* Get board and servo ids */
    sd21_t dev = atoi(argv[1]);
    uint8_t servo_id = atoi(argv[2]) - 1;

    /* Display usage */
    sd21_calib_print_usage(dev, servo_id);

    puts("");
    puts("=======================");
    puts("");

    /* Display current servo name */
    puts(sd21_servo_get_name(dev, servo_id));

    /* Key pressed */
    char c[2];

    while (c[0] != 'q') {
        uint16_t current_position = 0;
        sd21_servo_get_position(dev, servo_id, &current_position);

        /* Print current position on same line */
        print_str("Current position: ");
        print_u32_dec(current_position);
        print_str("    \r");

        c[0] = getchar();

        /* useful only for 0 to 9 keys */
        uint8_t defined_position = (uint8_t)atoi(c);

        switch(c[0]) {
            /* Opened position */
            case 'o':
                sd21_servo_reach_position(dev, servo_id, SD21_SERVO_POS_OPEN);
                break;
            /* Closed position */
            case 'c':
                sd21_servo_reach_position(dev, servo_id, SD21_SERVO_POS_CLOSE);
                break;
            /* Next servomotor */
            case 'n':
                servo_id = (servo_id >= sd21_config[dev].servos_nb - 1) ?
                    sd21_config[dev].servos_nb - 1 : servo_id + 1;
                printf("\n\n%s\n", sd21_servo_get_name(dev, servo_id));
                break;
            /* Previous servomotor */
            case 'p':
                servo_id = (servo_id == 0) ? 0 : servo_id - 1;
                printf("\n\n%s\n", sd21_servo_get_name(dev, servo_id));
                break;
            /* Reset position to middle (between opened and closed) */
            case 'r':
                sd21_servo_control_position(dev, servo_id, SD21_SERVO_POS_MID);
                break;
            /* Add SD21_SERVO_POS_STEP */
            case '+':
                sd21_servo_control_position(dev, servo_id, current_position
                        + SD21_SERVO_POS_STEP);
                break;
            /* Substract SD21_SERVO_POS_STEP */
            case '-':
                sd21_servo_control_position(dev, servo_id, current_position
                        - SD21_SERVO_POS_STEP);
                break;
            /* Intermediate positions */
            case '0':   /* SD21_SERVO_POS_OPEN */
            case '1':   /* SD21_SERVO_POS_CLOSE */
            case '2':
            case '3':
            case '4':
            case '5':
            case '6':
            case '7':
            case '8':
            case '9':
                if (defined_position < SD21_SERVO_POS_NUMOF)
                    sd21_servo_reach_position(dev, servo_id, defined_position);
                break;
            default:
                continue;
        }

    }

    puts("");

    return ret;

sd21_calib_servo_cmd_err:
    return ret;
}


void sd21_calib_init(const sd21_conf_t* sd21_config_new)
{
    sd21_config = sd21_config_new;

    sd21_numof = sizeof(*sd21_config) / sizeof(sd21_config[0]);

    shell_command_t cmd = { 
        "sc", "sd21_calib <board_id> <servo_id>",
        sd21_calib_servo_cmd
    };

    pf_add_shell_command(&cmd);
}
