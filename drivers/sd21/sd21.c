#include <stdio.h>

#include "platform.h"
#include "sd21.h"
#include "xtimer.h"

static uint16_t sd21_servo_positions[SD21_NUMOF][SD21_SERVO_NUMOF];

static void sd21_check_config(sd21_t dev, uint8_t servo_id)
{
    assert(dev < SD21_NUMOF);

    const sd21_conf_t *sd21 = &sd21_config[dev];

    assert(servo_id < sd21->servos_nb);
}

static int sd21_send_twi_cmd(sd21_t dev, uint8_t servo_id, uint8_t speed, uint16_t position)
{
    const sd21_conf_t *sd21 = &sd21_config[dev];

    assert(servo_id < sd21->servos_nb);

    uint8_t reg = (servo_id) * 3;
    uint8_t data[2];
    int ret = 0;

    irq_disable();

    for (int i = SD21_I2C_RETRIES; i > 0; i--) {
        ret = i2c_acquire(sd21->i2c_dev_id);
        if (!ret)
            break;
    }
    if (ret) goto sd21_send_twi_cmd_err;

    data[0] = reg;
    data[1] = speed;
    for (int i = SD21_I2C_RETRIES; i > 0; i--) {
        ret = i2c_write_bytes(sd21->i2c_dev_id, sd21->i2c_address, data, 2, 0);
        if (!ret)
            break;
        xtimer_usleep(20 * US_PER_MS);
    }
    if (ret) goto sd21_send_twi_cmd_err;

    data[0] = reg + 1;
    data[1] = (uint8_t) (position & 0x00FF);
    for (int i = SD21_I2C_RETRIES; i > 0; i--) {
        ret = i2c_write_bytes(sd21->i2c_dev_id, sd21->i2c_address, data, 2, 0);
        if (!ret)
            break;
        xtimer_usleep(20 * US_PER_MS);
    }
    if (ret) goto sd21_send_twi_cmd_err;

    data[0] = reg + 2;
    data[1] = position >> 8;
    for (int i = SD21_I2C_RETRIES; i > 0; i--) {
        ret = i2c_write_bytes(sd21->i2c_dev_id, sd21->i2c_address, data, 2, 0);
        if (!ret)
            break;
        xtimer_usleep(20 * US_PER_MS);
    }
    if (ret) goto sd21_send_twi_cmd_err;

    sd21_servo_positions[dev][servo_id] = position;

    for (int i = SD21_I2C_RETRIES; i > 0; i--) {
        ret = i2c_release(sd21->i2c_dev_id);
        if (!ret)
            break;
    }
    if (ret) goto sd21_send_twi_cmd_err;

    irq_enable();

    return 0;

sd21_send_twi_cmd_err:
    return -1;
}

int sd21_servo_control(sd21_t dev, uint8_t servo_id, uint8_t speed,
        uint16_t position)
{
    sd21_check_config(dev, servo_id);

    /* Limit position */
    if (position < SD21_SERVO_POS_MIN)
        position = SD21_SERVO_POS_MIN;
    if (position > SD21_SERVO_POS_MAX)
        position = SD21_SERVO_POS_MAX;

    return sd21_send_twi_cmd(dev, servo_id, speed, position);
}

int sd21_servo_reach_position(sd21_t dev, uint8_t servo_id, uint8_t pos_index)
{
    assert(pos_index < SD21_SERVO_POS_NUMOF);

    sd21_check_config(dev, servo_id);

    const sd21_conf_t *sd21 = &sd21_config[dev];

    const sd21_servo_t *servo = &sd21->servos[servo_id];

    return sd21_servo_control(dev, servo_id, servo->default_speed,
            servo->positions[pos_index]);
}

int sd21_servo_reset_position(sd21_t dev, uint8_t servo_id)
{

    sd21_check_config(dev, servo_id);

    const sd21_conf_t *sd21 = &sd21_config[dev];

    const sd21_servo_t *servo = &sd21->servos[servo_id];

    assert(servo->default_position < SD21_SERVO_POS_NUMOF);

    return sd21_servo_control(dev, servo_id, servo->default_speed,
            servo->positions[servo->default_position]);
}

uint16_t sd21_servo_get_position(sd21_t dev, uint8_t servo_id)
{
    sd21_check_config(dev, servo_id);

    return sd21_servo_positions[dev][servo_id];
}

const char* sd21_servo_get_name(sd21_t dev, uint8_t servo_id)
{
    sd21_check_config(dev, servo_id);

    const sd21_conf_t *sd21 = &sd21_config[dev];

    const sd21_servo_t *servo = &sd21->servos[servo_id];

    return servo->name;
}


void sd21_init(void)
{
    for (sd21_t dev = 0; dev < SD21_NUMOF; dev++) {
        xtimer_usleep(250 * US_PER_MS);
        /* Close all servomotors */
        for (uint8_t servo_id = 0; servo_id < sd21_config[dev].servos_nb;
                servo_id++) {
                if (sd21_servo_reset_position(dev, servo_id))
                    printf("Servo %u from board %u init failed !", servo_id,
                            dev);
                /* Wait a small tempo to avoid current peak */
                xtimer_usleep(50 * US_PER_MS);
        }
    }
}
