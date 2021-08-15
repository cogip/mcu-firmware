/* Standard includes */
#include <cstdio>
#include <cstdlib>

/* RIOT includes */
#include "fmt.h"

/* Project includes */
#define ENABLE_DEBUG        (0)
#include "debug.h"
#include "platform.hpp"
#include "sd21.h"
#include "shell_menu.hpp"
#include "shell_sd21.h"
#include "utils.h"
#include "tracefd/tracefd.hpp"

static const sd21_conf_t *sd21_config = NULL;

/* Board and servo ids */
static sd21_t dev = 0;
static uint8_t servo_id = 0;

static uint16_t sd21_get_current_position(void)
{
    uint16_t current_position = 0;

    sd21_servo_get_position(dev, servo_id, &current_position);

    /* Print current position on same line */
    cogip::tracefd::out.logf("Current position: %u", current_position);

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
        cogip::tracefd::out.logf("Bad number of arguments!");

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
    cogip::tracefd::out.logf("Current servo: %s", sd21_servo_get_name(dev, servo_id));

    return EXIT_SUCCESS;
}

static int sd21_cmd_previous_servo_cb(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    sd21_get_current_position();
    servo_id = (servo_id == 0) ? 0 : servo_id - 1;
    cogip::tracefd::out.logf("Current servo: %s", sd21_servo_get_name(dev, servo_id));

    return EXIT_SUCCESS;
}

static int sd21_cmd_reset_cb(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    sd21_servo_control_position(dev, servo_id, SD21_SERVO_POS_MID);

    sd21_get_current_position();

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
        cogip::tracefd::out.logf("Bad number of arguments!");
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

extern "C"
void sd21_shell_init(const sd21_conf_t *sd21_config_new)
{
    /* SD21 new_config */
    sd21_config = sd21_config_new;

    /* SD21 menu and commands */
    cogip::shell::menu *menu = new cogip::shell::menu(
        "SD21 menu", "sd21_menu", &cogip::shell::root_menu);

    menu->push_back(new cogip::shell::command(
        "d", "sd21 device <d>", sd21_cmd_device_cb));
    menu->push_back(new cogip::shell::command(
        "o", "Opened position", sd21_cmd_opened_cb));
    menu->push_back(new cogip::shell::command(
        "c", "Closed position", sd21_cmd_closed_cb));
    menu->push_back(new cogip::shell::command(
        "n", "Next servomotor", sd21_cmd_next_servo_cb));
    menu->push_back(new cogip::shell::command(
        "p", "Previous servomotor", sd21_cmd_previous_servo_cb));
    menu->push_back(new cogip::shell::command(
        "r", "Reset to center position", sd21_cmd_reset_cb));
    menu->push_back(new cogip::shell::command(
        "+", "Add " STR(SD21_SERVO_POS_STEP) "microseconds to current position", sd21_cmd_add_cb));
    menu->push_back(new cogip::shell::command(
        "-", "Substract " STR(SD21_SERVO_POS_STEP) "microseconds from current position", sd21_cmd_sub_cb));
    menu->push_back(new cogip::shell::command(
        "switch", "Switch to predefined position <n> (n between 0 and 9)", sd21_cmd_switch_cb));
}
