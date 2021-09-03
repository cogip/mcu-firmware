#pragma once

// Project includes
#include "obstacles/obstacles.hpp"

#include "sd21.h"

/*
 * Machine parameters
 */

#define USART_CONSOLE   USARTC0

/**********/
/* Servos */
/**********/

/* Arms */
#define APP_SERVO_ARM_LEFT              0, 0
#define APP_SERVO_ARM_RIGHT             0, 1

/* Buoys */
#define APP_SERVO_BUOY_EXTENDER_LEFT    0, 2
#define APP_SERVO_BUOY_EXTENDER_RIGHT   0, 3
#define APP_SERVO_BUOY_HOLDER_LEFT      0, 4
#define APP_SERVO_BUOY_HOLDER_RIGHT     0, 5

/* Flag */
#define APP_SERVO_FLAG                  0, 6

/*****************/
/* Servos states */
/*****************/

static const sd21_conf_t sd21_config_app[] = {
    {   /* SD12 ID0 */
        .i2c_dev_id = 1,
        .i2c_address = (0xC2 >> 1),
        .i2c_speed_khz = I2C_SPEED_NORMAL,

        .servos_nb = 7,
        .servos = {
            /* Servo 0-1 */
            {
                {                          // .positions
                    1950,   /* Open */
                    1050,   /* Closed */
                },
                SD21_SERVO_POS_CLOSE,      // .default_position
                0,                         // .default_speed
                "S0-1: Left buoy extender" // .name
            },
            /* Servo 0-2 */
            {
                {                          // .positions
                    1050,   /* Open */
                    1950,   /* Closed */
                },
                SD21_SERVO_POS_CLOSE,       // .default_position
                0,                          // .default_speed
                "S0-2: Right buoy extender" // .name
            },
            /* Servo 0-3 */
            {
                {                        // .positions
                    1700,   /* Open */
                    1600,   /* Closed */
                },
                SD21_SERVO_POS_OPEN,     // .default_position
                0,                       // .default_speed
                "S0-3: Left buoy holder" // .name
            },
            /* Servo 0-4 */
            {
                {                         // .positions
                    1350,   /* Open */
                    1450,   /* Closed */
                },
                SD21_SERVO_POS_OPEN,      // .default_position
                0,                        // .default_speed
                "S0-4: Right buoy holder" // .name
            },
            /* Servo 0-5 */
            {
                {                         // .positions
                    1000,   /* Open */
                    1975,   /* Closed */
                },
                SD21_SERVO_POS_CLOSE,     // .default_position
                0,                        // .default_speed
                "S0-5: Left arm"          // .name
            },
            /* Servo 0-6 */
            {
                {                         // .positions
                    2000,   /* Open */
                    950,    /* Closed */
                },
                SD21_SERVO_POS_CLOSE,     // .default_position
                0,                        // .default_speed
                "S0-6: Right arm"         // .name
            },
            /* Servo 0-7 */
            {
                {                         // .positions
                    1000,   /* Open */
                    2000,   /* Closed */
                },
                SD21_SERVO_POS_CLOSE,     // .default_position
                0,                        // .default_speed
                "S0-7: Flag"              // .name
            },
        },
    },
};

/// @brief Return a polygon obstacle delimitng the table borders.
/// @return                      table borders
const cogip::obstacles::Polygon *app_get_borders(void);

void app_init(void);
void app_init_tasks(void);
