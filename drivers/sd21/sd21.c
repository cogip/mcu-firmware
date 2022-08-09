/* System includes */
#include <errno.h>
#include <stdio.h>

/* RIOT includes */
#include <ztimer.h>

/* Project includes */
#include "sd21.h"

#ifdef MODULE_SHELL_SD21
#include "shell_sd21.h"
#endif /* MODULE_SHELL_SD21 */

static const sd21_conf_t *sd21_config = NULL;
static size_t sd21_numof = 0;

static const sd21_servo_t *sd21_get_servo(sd21_t dev, uint8_t servo_id)
{
    assert(sd21_config != NULL);

    assert(dev < sd21_numof);

    const sd21_conf_t *sd21 = &sd21_config[dev];

    assert(servo_id < sd21->servos_nb);

    return &sd21->servos[servo_id];
}

static int sd21_write_twi_cmd(sd21_t dev, uint8_t servo_id,
                              const uint16_t *data, uint8_t offset)
{
    const sd21_conf_t *sd21 = &sd21_config[dev];

    uint8_t reg = (servo_id) * 3 + offset;
    int ret = 0;

    irq_disable();

    ret = i2c_acquire(sd21->i2c_dev_id);
    if (ret) {
        goto sd21_write_twi_cmd_err;
    }

    ret = i2c_write_regs(sd21->i2c_dev_id, sd21->i2c_address, reg, data, 2,
                         0);
    if (ret) {
        goto sd21_write_twi_cmd_err;
    }

    i2c_release(sd21->i2c_dev_id);

    irq_enable();

    return 0;

sd21_write_twi_cmd_err:
    irq_enable();
    return -1;
}

static int sd21_read_twi_cmd(sd21_t dev, uint8_t servo_id, uint16_t *data,
                             uint8_t offset)
{
    const sd21_conf_t *sd21 = &sd21_config[dev];
    uint8_t tmp[3]; /* 1 byte for address + 2 bytes for data */

    uint8_t reg = (servo_id) * 3 + offset;
    int ret = 0;

    irq_disable();

    ret = i2c_acquire(sd21->i2c_dev_id);
    if (ret) {
        goto sd21_read_twi_cmd_err;
    }

    ret = i2c_read_regs(sd21->i2c_dev_id, sd21->i2c_address, reg, tmp,
                        3, 0);

    if (ret) {
        goto sd21_read_twi_cmd_err;
    }

    *data = (tmp[2] << 8) + tmp[1];

    i2c_release(sd21->i2c_dev_id);

    irq_enable();

sd21_read_twi_cmd_err:
    irq_enable();
    return -1;
}

int sd21_servo_control_position(sd21_t dev, uint8_t servo_id,
                                uint16_t position)
{
    /* Limit position */
    if (position < SD21_SERVO_POS_MIN) {
        position = SD21_SERVO_POS_MIN;
    }
    if (position > SD21_SERVO_POS_MAX) {
        position = SD21_SERVO_POS_MAX;
    }

    return sd21_write_twi_cmd(dev, servo_id, &position, 1);
}

int sd21_servo_reach_position(sd21_t dev, uint8_t servo_id, uint8_t pos_index)
{
    assert(pos_index < SD21_SERVO_POS_NUMOF);

    const sd21_servo_t *servo = sd21_get_servo(dev, servo_id);

    return sd21_servo_control_position(dev, servo_id,
                                       servo->positions[pos_index]);
}

int sd21_servo_reset_position(sd21_t dev, uint8_t servo_id)
{
    const sd21_servo_t *servo = sd21_get_servo(dev, servo_id);

    assert(servo->default_position < SD21_SERVO_POS_NUMOF);

    return sd21_servo_control_position(dev, servo_id,
                                       servo->positions[servo->default_position]);
}

int sd21_servo_get_position(sd21_t dev, uint8_t servo_id, uint16_t *position)
{
    if (!position) {
        return -EINVAL;
    }

    sd21_get_servo(dev, servo_id);

    return sd21_read_twi_cmd(dev, servo_id, (void *)position, 1);
}

const char *sd21_servo_get_name(sd21_t dev, uint8_t servo_id)
{
    const sd21_servo_t *servo = sd21_get_servo(dev, servo_id);

    return servo->name;
}

void sd21_init(const sd21_conf_t *sd21_config_new)
{
    sd21_config = sd21_config_new;

    sd21_numof = sizeof(*sd21_config) / sizeof(sd21_config[0]);

    for (sd21_t dev = 0; dev < sd21_numof; dev++) {
        ztimer_sleep(ZTIMER_MSEC, 250);
        /* Close all servomotors */
        for (uint8_t servo_id = 0; servo_id < sd21_config[dev].servos_nb;
             servo_id++) {
            if (sd21_servo_reset_position(dev, servo_id)) {
                printf("Servo %u from board %u init failed !", servo_id,
                       dev);
            }
            /* Wait a small tempo to avoid current peak */
            ztimer_sleep(ZTIMER_MSEC, 50);
        }
    }

#ifdef MODULE_SHELL_SD21
    sd21_shell_init(sd21_config);
#endif
}
