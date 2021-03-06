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
#include "shell_menu.h"
#include "shell_sd21.h"
#include "utils.h"

static const sd21_conf_t *sd21_config = NULL;

/* Board and servo ids */
static sd21_t dev = 0;
static uint8_t servo_id = 0;

static uint16_t sd21_get_current_position(void)
{
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

static int sd21_cmd_device_cb(int argc, char **argv)
{
    (void)argc;
    (void)argv;


    /* Check arguments */
    if (argc != 2) {
        puts("Bad number of arguments!");

        return EXIT_FAILURE;
    }

    dev = atoi(argv[1]);

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

    switch (defined_position) {
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

void sd21_shell_init(const sd21_conf_t *sd21_config_new)
{
    /* SD21 new_config */
    sd21_config = sd21_config_new;

    /* SD21 menu and commands */
    shell_menu_t menu = menu_init("SD21 menu", "sd21_menu", menu_root, NULL);

    const shell_command_t shell_sd21_menu_commands[] = {
        { "d", "sd21 device", sd21_cmd_device_cb },
        { "o", "Opened position", sd21_cmd_opened_cb },
        { "c", "Closed position", sd21_cmd_closed_cb },
        { "n", "Next servomotor", sd21_cmd_next_servo_cb },
        { "p", "Previous servomotor", sd21_cmd_previous_servo_cb },
        { "r", "Reset to center position", sd21_cmd_reset_cb },
        { "+", "Add "STR (SD21_SERVO_POS_STEP)"microseconds to current position",
          sd21_cmd_add_cb },
        { "-", "Substract "STR (SD21_SERVO_POS_STEP) "microseconds \
            from current position", sd21_cmd_sub_cb },
        { "switch", "Switch to predefined position <n> (n between 0 and 9)",
          sd21_cmd_switch_cb },
        MENU_NULL_CMD,
    };

    menu_add_list(menu, shell_sd21_menu_commands);
}
