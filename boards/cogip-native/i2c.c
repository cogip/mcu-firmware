#include "periph/i2c.h"

void i2c_init(i2c_t dev)
{
    (void)dev;
}

void i2c_acquire(i2c_t dev)
{
    (void)dev;
}

void i2c_release(i2c_t dev)
{
    (void)dev;
}

int i2c_read_bytes(i2c_t dev, uint16_t addr,
                   void *data, size_t len, uint8_t flags)
{
    (void)dev;
    (void)addr;
    (void)data;
    (void)len;
    (void)flags;

    return 0;
}

int i2c_write_bytes(i2c_t dev, uint16_t addr, const void *data,
                    size_t len, uint8_t flags)
{
    (void)dev;
    (void)addr;
    (void)data;
    (void)len;
    (void)flags;

    return 0;
}
