#include <stdio.h>

#include "platform.h"
#include "sd21.h"

#define SD21_ADDRESS    (0xC2 >> 1)

#define REG_VERSION 64
#define REG_VOLTAGE 65

/* calibration constants */
#define CAL_RST 1600
#define CAL_MIN 600
#define CAL_MAX 2600

/**
 * Blue : min = 800 - max = 2400
 * Tower Pro : min = 600 - max = 2400
 * Emax : min = 600 - max = 2600
 */
/**
 * \fn void sd21_send (uint8_t servo, uint8_t speed, uint16_t position)
 * \brief
 * \param servo : servo number (to 1 from 21)
 * \param speed : servo speed (0 is the maximum speed)
 * \param position : pulse width in us
 */
static void sd21_send_twi_cmd(i2c_t dev_id, uint8_t servo, uint8_t speed, uint16_t position)
{
    uint8_t reg = (servo) * 3;
    uint8_t data[2];

    i2c_acquire(dev_id);

    data[0] = reg;
    data[1] = speed;
    i2c_write_bytes(dev_id, SD21_ADDRESS, data, 2, 0);

    data[0] = reg + 1;
    data[1] = (uint8_t) (position & 0x00ff);
    i2c_write_bytes(dev_id, SD21_ADDRESS, data, 2, 0);

    data[0] = reg + 2;
    data[1] = position >> 8;
    i2c_write_bytes(dev_id, SD21_ADDRESS, data, 2, 0);

    i2c_release(dev_id);
}

void sd21_control_servo(sd21_t *obj, uint8_t servo_id, uint8_t position)
{
    uint16_t value;

    switch (position) {
        case SD21_SERVO_OPEN:
            value = obj->servos[servo_id].value_open;
            break;
        case SD21_SERVO_CLOSE:
            value = obj->servos[servo_id].value_close;
            break;
        default:
            printf("Invalid position\n");
            return;
    }

    sd21_send_twi_cmd(obj->bus_id, servo_id, 0, value);
}

#if defined(MODULE_CALIBRATION)
static void sd21_calibration_usage(sd21_t *obj)
{
    cons_printf("\n>>> Entering sd21 calibration\n\n");

    cons_printf("servos_nb = %d\n\n", obj->servos_nb);

    cons_printf("\t'd' to dump all settings\n");
    cons_printf("\t'n' to select next servo\n");
    cons_printf("\t'b' to select prev servo\n");
    cons_printf("\t's' to switch current setting (init, open or close)\n");
    cons_printf("\t'r' to reset current setting to %d\n", CAL_RST);
    cons_printf("\t'+' to add 25\n");
    cons_printf("\t'-' to sub 25\n");
    cons_printf("\n");
    cons_printf("\t'O' to open ALL servos\n");
    cons_printf("\t'C' to close ALL servos\n");
    cons_printf("\n");
    cons_printf("\t'h' to display this help\n");
    cons_printf("\t'q' to quit\n");
    cons_printf("\n");
}


static void sd21_calibration_dump(sd21_t *obj)
{
    uint8_t i;

    cons_printf("\n\t\tinit\topen\tclose\n");

    for (i = 0; i < obj->servos_nb; i++) {
        uint16_t value;

        cons_printf("servo #%02d\t", i);

        value = obj->servos[i].value_init;
        cons_printf("%4d\t", value);
        value = obj->servos[i].value_open;
        cons_printf("%4d\t", value);
        value = obj->servos[i].value_close;
        cons_printf("%4d\n", value);
    }
}

void sd21_enter_calibration(sd21_t *obj)
{
    int c, i;
    uint8_t quit = 0;
    static uint8_t servo_id = 0;
    enum { SET_INIT = 0, SET_OPEN, SET_CLOSE } servo_setting = SET_INIT;
    const char *setting_str[] = { "init", "open", "close" };

    sd21_calibration_usage(obj);

    while (!quit) {
        uint16_t *cur;

        switch (servo_setting) {
            default:
            case SET_INIT:
                cur = &obj->servos[servo_id].value_init;
                break;
            case SET_OPEN:
                cur = &obj->servos[servo_id].value_open;
                break;
            case SET_CLOSE:
                cur = &obj->servos[servo_id].value_close;
                break;
        }

        /* display prompt */
        cons_printf("[%02d].value_%s = %4d $ ",
                    servo_id,
                    setting_str[servo_setting],
                    *cur);

        /* wait for command */
        c = cons_getchar();
        cons_printf("%c\n", c);

        switch (c) {
            case 'd':
                sd21_calibration_dump(obj);
                break;
            case 'n':
                servo_id += 1;
                servo_id %= obj->servos_nb;
                break;
            case 'b':
                if (servo_id) {
                    servo_id -= 1;
                }
                else {
                    servo_id = obj->servos_nb - 1;
                }
                break;
            case 's':
                servo_setting += 1;
                servo_setting %= SET_CLOSE + 1;
                break;
            case 'r':
                *cur = CAL_RST;
                sd21_send_twi_cmd(obj->bus_id, servo_id, 0, *cur);
                break;
            case '+':
                *cur = *cur + 25 > CAL_MAX ? CAL_MAX : *cur + 25;
                sd21_send_twi_cmd(obj->bus_id, servo_id, 0, *cur);
                break;
            case '-':
                *cur = *cur - 25 < CAL_MIN ? CAL_MIN : *cur - 25;
                sd21_send_twi_cmd(obj->bus_id, servo_id, 0, *cur);
                break;
            case 'O':
                for (i = 0; i < obj->servos_nb; i++) {
                    uint16_t value = obj->servos[i].value_open;

                    if (value) {
                        sd21_send_twi_cmd(obj->bus_id, i, 0, value);
                    }
                }
                break;
            case 'C':
                for (i = 0; i < obj->servos_nb; i++) {
                    uint16_t value = obj->servos[i].value_close;

                    if (value) {
                        sd21_send_twi_cmd(obj->bus_id, i, 0, value);
                    }
                }
                break;
            case 'h':
                sd21_calibration_usage(obj);
                break;
            case 'q':
                quit = 1;
                break;
            default:
                cons_printf("\n");
                break;
        }
    }
}
#endif /* MODULE_CALIBRATION */

/**
 */
void sd21_setup(sd21_t *obj)
{
    i2c_init(obj->bus_id);

//#if defined(CONFIG_SD21_INIT_AT_STARTUP)
    for (uint8_t i = 0; i < obj->servos_nb; i++) {
        uint16_t value_init = obj->servos[i].value_init;

        if (value_init) {
            sd21_send_twi_cmd(obj->bus_id, i, 0, value_init);
        }
    }
//#endif
}
