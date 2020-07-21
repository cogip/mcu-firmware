#include <stdio.h>

/* Standard includes */
#include <stdlib.h>

/* RIOT includes */
#include "fmt.h"

/* Project includes */
#define ENABLE_DEBUG        (0)
#include "debug.h"
#include "platform.h"
#include "sd21.h"
#include "calibration/calib_sd21.h"

static const sd21_conf_t* sd21_config = NULL;
static size_t sd21_numof = 0;

/* Shell command array */
static shell_command_linked_t sd21_shell_commands;
static const char *sd21_name = "sd21";

/* Board and servo ids */
static sd21_t dev = 0;
static uint8_t servo_id = 0;

static uint16_t sd21_get_current_position(void) {
    uint16_t current_position = 0;
    sd21_servo_get_position(dev, servo_id, &current_position);

    /* Print current position on same line */
    print_str("Current position: ");
    print_u32_dec(current_position);
    print_str("    \r");

    return current_position;
}

static int sd21_cmd_opened_cb(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    sd21_get_current_position();
    sd21_servo_reach_position(dev, servo_id, SD21_SERVO_POS_OPEN);

    return EXIT_SUCCESS;
}

static int sd21_cmd_closed_cb(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    sd21_get_current_position();
    sd21_servo_reach_position(dev, servo_id, SD21_SERVO_POS_CLOSE);

    return EXIT_SUCCESS;
}

static int sd21_cmd_next_servo_cb(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    sd21_get_current_position();
    servo_id = (servo_id >= sd21_config[dev].servos_nb - 1) ?
        sd21_config[dev].servos_nb - 1 : servo_id + 1;
    printf("\n\n%s\n", sd21_servo_get_name(dev, servo_id));

    return EXIT_SUCCESS;
}

static int sd21_cmd_previous_servo_cb(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    sd21_get_current_position();
    servo_id = (servo_id == 0) ? 0 : servo_id - 1;
    printf("\n\n%s\n", sd21_servo_get_name(dev, servo_id));

    return EXIT_SUCCESS;
}

static int sd21_cmd_reset_cb(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    sd21_get_current_position();
    sd21_servo_control_position(dev, servo_id, SD21_SERVO_POS_MID);

    return EXIT_SUCCESS;
}

static int sd21_cmd_add_cb(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    uint16_t current_position = sd21_get_current_position();
    sd21_servo_control_position(dev, servo_id, current_position + SD21_SERVO_POS_STEP);

    return EXIT_SUCCESS;
}

static int sd21_cmd_sub_cb(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    uint16_t current_position = sd21_get_current_position();
    sd21_servo_control_position(dev, servo_id, current_position - SD21_SERVO_POS_STEP);

    return EXIT_SUCCESS;
}

static int sd21_cmd_switch_cb(int argc, char **argv)
{
    sd21_get_current_position();

    /* Check arguments */
    if (argc != 2) {
        puts("Bad number of arguments!");
        return EXIT_FAILURE;
    }

    uint8_t defined_position = (uint8_t)atoi(argv[1]);

    switch (defined_position)
    {
        case 0: /* SD21_SERVO_POS_OPEN */
        case 1: /* SD21_SERVO_POS_CLOSE */
        case 2:
        case 3:
        case 4:
        case 5:
        case 6:
        case 7:
        case 8:
        case 9:
            if (defined_position < SD21_SERVO_POS_NUMOF) {
                sd21_servo_reach_position(dev, servo_id, defined_position);
            }
            break;
        default:
            break;
    }

    return EXIT_SUCCESS;
}

static int sd21_calib_servo_cmd(int argc, char **argv)
{
    (void)argv;
    int ret = 0;

    /* Check arguments */
    if (argc != 3) {
        puts("Bad number of arguments!");
        ret = -1;
        goto sd21_calib_servo_cmd_err;
    }

    /* Get board and servo ids */
    dev = atoi(argv[1]);
    servo_id = atoi(argv[2]) - 1;

    /* Display current servo name */
    puts(sd21_servo_get_name(dev, servo_id));

    pf_init_shell_commands(&sd21_shell_commands, sd21_name);

    pf_add_shell_command(&sd21_shell_commands, &cmd_exit_shell);

    shell_command_t sd21_cmd_opened = {"n", "Opened position", sd21_cmd_opened_cb};
    pf_add_shell_command(&sd21_shell_commands, &sd21_cmd_opened);

    shell_command_t sd21_cmd_closed = {"c", "Closed position", sd21_cmd_closed_cb};
    pf_add_shell_command(&sd21_shell_commands, &sd21_cmd_closed);

    shell_command_t sd21_cmd_next_servo = {"n", "Next servomotor", sd21_cmd_next_servo_cb};
    pf_add_shell_command(&sd21_shell_commands, &sd21_cmd_next_servo);

    shell_command_t sd21_cmd_previous_servo = {"n", "Previous servomotor", sd21_cmd_previous_servo_cb};
    pf_add_shell_command(&sd21_shell_commands, &sd21_cmd_previous_servo);

    shell_command_t sd21_cmd_reset = {"r", "Reset to center position", sd21_cmd_reset_cb};
    pf_add_shell_command(&sd21_shell_commands, &sd21_cmd_reset);

    #define STR(x) #x
    shell_command_t sd21_cmd_add = {"+", "Add "STR(SD21_SERVO_POS_STEP)"microseconds to current position", sd21_cmd_add_cb};
    pf_add_shell_command(&sd21_shell_commands, &sd21_cmd_add);

    shell_command_t sd21_cmd_sub = {"-", "Substract "STR(SD21_SERVO_POS_STEP)"microseconds from current position", sd21_cmd_sub_cb};
    pf_add_shell_command(&sd21_shell_commands, &sd21_cmd_sub);
    #undef STR

    shell_command_t sd21_cmd_switch = {"switch", "Switch to predefined position <n> (n between 0 and 9)", sd21_cmd_switch_cb};
    pf_add_shell_command(&sd21_shell_commands, &sd21_cmd_switch);

    /* Push new menu */
    DEBUG("sd21: Start shell\n");
    pf_push_shell_commands(&sd21_shell_commands);

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

    pf_add_shell_command(&pf_shell_commands, &cmd);
}
