/*
 * Copyright (C) 2019 COGIP Robotics association <cogip35@gmail.com>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/*
 * @{
 *
 * @file
 * @brief       vl53l0x-api RIOT interface
 *
 * @author      Gilles DOFFE <g.doffe@gmail.com>
 */

/* Project includes */
#include "board.h"
#include "vl53l0x.h"

static VL53L0X_Dev_t devices[VL53L0X_NUMOF];
static VL53L0X_Error status[VL53L0X_NUMOF];

int vl53l0x_init_dev(vl53l0x_t dev)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    VL53L0X_Dev_t *st_api_vl53l0x = NULL;
    uint32_t refSpadCount;
    uint8_t isApertureSpads;
    uint8_t VhvSettings;
    uint8_t PhaseCal;

    status[dev] = VL53L0X_ERROR_UNDEFINED;

    /* Check device exists */
    assert(dev < VL53L0X_NUMOF);

    /* Get device description (not yet initialized) */
    st_api_vl53l0x = &devices[dev];

    /* Retrieve ToF */
    const vl53l0x_conf_t *vl53l0x = &vl53l0x_config[dev];

    st_api_vl53l0x->I2cDev =  vl53l0x->i2c_dev;
    st_api_vl53l0x->I2cDevAddr =  vl53l0x->i2c_addr;

    /* Force use of I2C communication protocol */
    st_api_vl53l0x->comms_type =  1;

    /* Data init */
    if (Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_DataInit(st_api_vl53l0x);
    }

    /* Static init */
    if (Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_StaticInit(st_api_vl53l0x);
    }

    /* Reference calibration */
    if (Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_PerformRefCalibration(st_api_vl53l0x,
                                               &VhvSettings, &PhaseCal);
    }

    /* Spad init */
    if (Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_PerformRefSpadManagement(st_api_vl53l0x,
                                                  &refSpadCount, &isApertureSpads);
    }

    if (Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_SetDeviceMode(st_api_vl53l0x,
                                       VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);
    }

    if (Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_StartMeasurement(st_api_vl53l0x);
    }

    status[dev] = Status;

    return Status;
}

int vl53l0x_reset_dev(vl53l0x_t dev)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    VL53L0X_Dev_t *st_api_vl53l0x = &devices[dev];

    status[dev] = VL53L0X_ERROR_UNDEFINED;

    /* Check device exists */
    assert(dev < VL53L0X_NUMOF);

    Status = VL53L0X_ResetDevice(st_api_vl53l0x);

    status[dev] = Status;

    return Status;
}

void vl53l0x_reset(void)
{
    for (vl53l0x_t dev = 0; dev < VL53L0X_NUMOF; dev++) {
        assert(vl53l0x_reset_dev(dev) == 0);
    }
}

void vl53l0x_init(void)
{
    for (vl53l0x_t dev = 0; dev < VL53L0X_NUMOF; dev++) {
        assert(vl53l0x_init_dev(dev) == 0);
    }
}

uint16_t vl53l0x_continuous_ranging_get_measure(vl53l0x_t dev)
{
    VL53L0X_RangingMeasurementData_t RangingMeasurementData;
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    FixPoint1616_t LimitCheckCurrent;
    VL53L0X_Dev_t *st_api_vl53l0x = NULL;

    /* Check device exists */
    assert(dev < VL53L0X_NUMOF);

    /* Get device description (not yet initialized) */
    st_api_vl53l0x = &devices[dev];

    /* Check device description is initialized */
    if (status[dev] == 0) {
        Status = VL53L0X_GetRangingMeasurementData(st_api_vl53l0x,
                                                   &RangingMeasurementData);

        VL53L0X_GetLimitCheckCurrent(st_api_vl53l0x,
                                     VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD,
                                     &LimitCheckCurrent);

        if (Status != VL53L0X_ERROR_NONE) {
            goto vl53l0x_continuous_ranging_get_measure_err;
        }
    }
    else {
        goto vl53l0x_continuous_ranging_get_measure_err;
    }

    return RangingMeasurementData.RangeMilliMeter;

vl53l0x_continuous_ranging_get_measure_err:
    return UINT16_MAX;
}
/** @} */
