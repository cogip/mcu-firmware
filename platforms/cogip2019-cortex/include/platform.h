#ifndef PLATFORM_H_
#define PLATFORM_H_

#define SD21_SERVO_NUMOF        12
#define SD21_SERVO_POS_NUMOF    3
#define SD21_SERVO_POS_MIN      500
#define SD21_SERVO_POS_MAX      2500

/* Project includes */
#include "platform-common.h"
#include "sd21.h"
#include "vl53l0x.h"

/*
 * Machine parameters
 */

/* To be computed :
 *  - PULSE_PER_MM		: Number of pulses per mm of coding wheel
 *  - WHEELS_DISTANCE		: Distance between coding wheels in pulses
 *  - PULSE_PER_DEGREE		: Number of pulses per degree of coding wheel
 *
 * Must be known :
 *  - WHEELS_DIAMETER		: Coding wheel diameter
 *  - WHEELS_DISTANCE_MM	: Distance between coding wheels in mm
 *
 * Must be known and defined :
 *  - WHEELS_ENCODER_RESOLUTION	: Number of pulses by turn of coding wheels
 */

#define WHEELS_ENCODER_RESOLUTION   2000
/* WHEELS_PERIMETER = pi*WHEELS_DIAMETER
 * PULSE_PER_MM = WHEELS_ENCODER_RESOLUTION / WHEELS_PERIMETER
 */
#define PULSE_PER_MM        10.61
/* WHEELS_DISTANCE = WHEELS_DISTANCE_MM * PULSE_PER_MM */
#define WHEELS_DISTANCE     2962.312
/* WHEELS_DISTANCE*2*pi pulses for 360 deg. Thus 51.76 pulses per deg */
#define PULSE_PER_DEGREE    51.70

#define MAX_ACC     10
#define MAX_SPEED   20

#define HBRIDGE_MOTOR_LEFT  0
#define HBRIDGE_MOTOR_RIGHT 1

#define QDEC_MODE   QDEC_X4
#define QDEC_LEFT_POLARITY      -1
#define QDEC_RIGHT_POLARITY     1

#define USART_CONSOLE   USARTC0

#define AVOIDANCE_BORDER_X_MIN  -2000
#define AVOIDANCE_BORDER_X_MAX  2000
#define AVOIDANCE_BORDER_Y_MIN  -2000
#define AVOIDANCE_BORDER_Y_MAX  2000

#define OBSTACLE_BORDER_X_MIN   -2000
#define OBSTACLE_BORDER_X_MAX   2000
#define OBSTACLE_BORDER_Y_MIN   -2000
#define OBSTACLE_BORDER_Y_MAX   2000

#define CTRL_BLOCKING_NB_ITERATIONS 200

#define GPIO_CAMP       GPIO_PIN(PORT_B, 1)
#define GPIO_STARTER    GPIO_PIN(PORT_B, 2)

static const ctrl_quadpid_parameters_t ctrl_quadpid_params = {
        .linear_speed_pid = {
            .kp = 15.,
            .ki = 2,
            .kd = 0.,
        },
        .angular_speed_pid = {
            .kp = 15.,
            .ki = 2,
            .kd = 0.,
        },
        .linear_pose_pid = {
            .kp = 0.05,
            .ki = 0.,
            .kd = 0,
        },
        .angular_pose_pid = {
            .kp = 0.1,
            .ki = 0.,
            .kd = 0.,
        },

        .min_distance_for_angular_switch = 3 /* mm */,
        .min_angle_for_pose_reached = 2 /* °deg */,
        .regul = CTRL_REGUL_POSE_DIST,
};

static const sd21_conf_t sd21_config[] = {
    {
        .i2c_dev_id = 0,
        .i2c_address = (0xC2 >> 1),
        .i2c_speed_khz = I2C_SPEED_FAST,

        .servos_nb = 12,
        .servos = {
            /* Servo 1 */
            {
                .positions = {
                    685, /* Ramp Disposal */
                    2285, /* Puck prehension */
                    1500, /* Neutral */
                },
                .default_position = 2,
                .default_speed = 0,
                .name = "S1: suction cup front left"
            },
            /* Servo 2 */
            {
                .positions = {
                    2300, /* Ramp Disposal */
                    675, /* Puck prehension */
                    1500, /* Neutral */
                },
                .default_position = 2,
                .default_speed = 0,
                .name = "S2: suction cup front central"
            },
            /* Servo 3 */
            {
                .positions = {
                    2450, /* Ramp Disposal */
                    575, /* Puck prehension */
                    1500, /* Neutral */
                },
                .default_position = 2,
                .default_speed = 0,
                .name = "S3: suction cup front right"
            },
            /* Servo 4 */
            {
                .positions = {
                    1260,   /* Open */
                    2000,   /* Close */
                    1500,   /* Default */
                },
                .default_position = 1,
                .default_speed = 0,
                .name = "S4: front right arm"
            },
            /* Servo 5 */
            {
                .positions = {
                    1060,   /* Top position */
                    2000,   /* Bottom position */
                    1500,   /* Default */
                },
                .default_position = SD21_SERVO_POS_CLOSE,
                .default_speed = 0,
                .name = "S5: front central elevator"
            },
            /* Servo 6 */
            {
                .positions = {
                    1100,   /* Top position */
                    2050,   /* Bottom position */
                    1500,   /* Default */
                },
                .default_position = SD21_SERVO_POS_CLOSE,
                .default_speed = 0,
                .name = "S6: front left elevator"
            },
            /* Servo 7 */
            {
                .positions = {
                    1675,   /* Open */
                    1125,   /* Close */
                    1500,   /* Default */
                },
                .default_position = SD21_SERVO_POS_CLOSE,
                .default_speed = 0,
                .name = "S7: front left arm"
            },
            /* Servo 8 */
            {
                .positions = {
                    1975,   /* Top position */
                    1075,   /* Bottom position */
                    1500,   /* Default */
                },
                .default_position = SD21_SERVO_POS_CLOSE,
                .default_speed = 0,
                .name = "S8: front right elevator"
            },
            /* Servo 9 */
            {
                .positions = {
                    1740,   /* Open */
                    1160,   /* Close */
                    1500,   /* Default */
                },
                .default_position = SD21_SERVO_POS_CLOSE,
                .default_speed = 0,
                .name = "S9: front ramp left disposal"
            },
            /* Servo 10 */
            {
                .positions = {
                    1700,   /* Right */
                    1520,   /* Left  */
                    1600,
                },
                .default_position = 2,
                .default_speed = 0,
                .name = "S10: front ramp rotation"
            },
            /* Servo 11 */
            {
                .positions = {
                    1260,   /* Open */
                    1880,   /* Close */
                    1500,   /* Default */
                },
                .default_position = SD21_SERVO_POS_CLOSE,
                .default_speed = 0,
                .name = "S11: front ramp right disposal"
            },
            /* Servo 12 */
            {
                .positions = {
                    960,    /* Open */
                    1900,   /* Close */
                    1500,   /* Default */
                },
                .default_position = SD21_SERVO_POS_OPEN,
                .default_speed = 0,
                .name = "S12: front ramp puck block"
            },
        },
    }
};

#define SD21_NUMOF     (sizeof(sd21_config) / sizeof(sd21_config[0]))

#endif /* PLATFORM_H_ */
