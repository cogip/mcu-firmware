#ifndef PLATFORM_H_
#define PLATFORM_H_

#define SD21_SERVO_NUMOF        12
#define SD21_SERVO_POS_NUMOF    3
#define SD21_SERVO_POS_MIN      500
#define SD21_SERVO_POS_MAX      2500

/* Project includes */
#include "board.h"
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

#define MAX_ACC     4
#define MAX_SPEED   10

#define HBRIDGE_MOTOR_LEFT  0
#define HBRIDGE_MOTOR_RIGHT 1

#define QDEC_MODE   QDEC_X4
#define QDEC_LEFT_POLARITY      -1
#define QDEC_RIGHT_POLARITY     1

#define PF_CTRL_BLOCKING_SPEED_TRESHOLD         2
#define PF_CTRL_BLOCKING_SPEED_ERR_TRESHOLD     1
#define PF_CTRL_BLOCKING_NB_ITERATIONS          10

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


/**********/
/* Servos */
/**********/

/* Suction cups */
#define PF_SERVO_FL_CUP             0, 0
#define PF_SERVO_FC_CUP             0, 1
#define PF_SERVO_FR_CUP             0, 2

#define PF_SERVO_BL_CUP             0, 11
#define PF_SERVO_BC_CUP             0, 10
#define PF_SERVO_BR_CUP             0, 9

/* Elevators */
#define PF_SERVO_FC_ELEVATOR        0, 4
#define PF_SERVO_FL_ELEVATOR        0, 5
#define PF_SERVO_FR_ELEVATOR        0, 7

#define PF_SERVO_BC_ELEVATOR        0, 7
#define PF_SERVO_BL_ELEVATOR        0, 8
#define PF_SERVO_BR_ELEVATOR        0, 6

/* Ramps */
#define PF_SERVO_FL_RAMP_DISP       0, 8
#define PF_SERVO_FR_RAMP_DISP       0, 10
#define PF_SERVO_F_RAMP             0, 9
#define PF_SERVO_F_RAMP_BLOCK       0, 11

#define PF_SERVO_BL_RAMP_DISP       1, 3
#define PF_SERVO_BR_RAMP_DISP       1, 4
#define PF_SERVO_B_RAMP             1, 5
#define PF_SERVO_B_RAMP_BLOCK       1, 2

/* ARMs */
#define PF_SERVO_FR_ARM             0, 3
#define PF_SERVO_FL_ARM             0, 6

/*****************/
/* Servos states */
/*****************/

/* Suction cups */
#define PF_SERVO_STATE_CUP_TAKE     1
#define PF_SERVO_STATE_CUP_HOLD     2
#define PF_SERVO_STATE_CUP_RAMP     0

typedef struct {
    double angle_offset;
    double distance_offset;
} pf_sensor_t;

static const pf_sensor_t pf_sensors[VL53L0X_NUMOF] = {
    {
        .angle_offset = -135,
        .distance_offset = 190,
    },
    {
        .angle_offset = 180,
        .distance_offset = 170,
    },
    {
        .angle_offset = 135,
        .distance_offset = 1900,
    },
    {
        .angle_offset = -45,
        .distance_offset = 190,
    },
    {
        .angle_offset = 0,
        .distance_offset = 170,
    },
    {
        .angle_offset = 45,
        .distance_offset = 190,
    },
};

static const ctrl_quadpid_parameters_t ctrl_quadpid_params = {
        .linear_speed_pid = {
            .kp = 150.,
            .ki = 2,
            .kd = 0.,
        },
        .angular_speed_pid = {
            .kp = 150.,
            .ki = 2,
            .kd = 0.,
        },
        .linear_pose_pid = {
            .kp = 1,
            .ki = 0.,
            .kd = 2,
        },
        .angular_pose_pid = {
            .kp = 1,
            .ki = 0.,
            .kd = 5,
        },

        .min_distance_for_angular_switch = 3,   // mm,
        .min_angle_for_pose_reached = 2,        // deg,
        .regul = CTRL_REGUL_POSE_DIST,
};

static const sd21_conf_t sd21_config[] = {
    {   /* SD12 ID0 */
        .i2c_dev_id = 0,
        .i2c_address = (0xC2 >> 1),
        .i2c_speed_khz = I2C_SPEED_FAST,

        .servos_nb = 12,
        .servos = {
            /* Servo 0-1 */
            {
                .positions = {
                    685,  /* Ramp Disposal */
                    2285, /* Puck prehension */
                    1500, /* Neutral */
                },
                .default_position = PF_SERVO_STATE_CUP_HOLD,
                .default_speed = 0,
                .name = "S0-1: front left suction cup"
            },
            /* Servo 0-2 */
            {
                .positions = {
                    2300, /* Ramp Disposal */
                    675,  /* Puck prehension */
                    1500, /* Neutral */
                },
                .default_position = PF_SERVO_STATE_CUP_HOLD,
                .default_speed = 0,
                .name = "S0-2: front central suction cup"
            },
            /* Servo 0-3 */
            {
                .positions = {
                    2450, /* Ramp Disposal */
                    575,  /* Puck prehension */
                    1400, /* Neutral */
                },
                .default_position = PF_SERVO_STATE_CUP_HOLD,
                .default_speed = 0,
                .name = "S0-3: front right suction cup"
            },
            /* Servo 0-4 */
            {
                .positions = {
                    1250,   /* Open */
                    2000,   /* Close */
                    1500,   /* Default */
                },
                .default_position = 1,
                .default_speed = 0,
                .name = "S0-4: front right arm"
            },
            /* Servo 0-5 */
            {
                .positions = {
                    1050,   /* Top position */
                    2000,   /* Bottom position */
                    1500,   /* Default */
                },
                .default_position = 1,
                .default_speed = 0,
                .name = "S0-5: front central elevator"
            },
            /* Servo 0-6 */
            {
                .positions = {
                    1100,   /* Top position */
                    2000,   /* Bottom position */
                    1500,   /* Default */
                },
                .default_position = 1,
                .default_speed = 0,
                .name = "S0-6: front left elevator"
            },
            /* Servo 0-7 */
            {
                .positions = {
                    1675,   /* Open */
                    1125,   /* Close */
                    1500,   /* Default */
                },
                .default_position = 1,
                .default_speed = 0,
                .name = "S0-7: front left arm"
            },
            /* Servo 0-8 */
            {
                .positions = {
                    1975,   /* Top position */
                    1075,   /* Bottom position */
                    1500,   /* Default */
                },
                .default_position = 1,
                .default_speed = 0,
                .name = "S0-8: front right elevator"
            },
            /* Servo 0-9 */
            {
                .positions = {
                    1750,   /* Open */
                    1150,   /* Close */
                    1500,   /* Default */
                },
                .default_position = 1,
                .default_speed = 0,
                .name = "S0-9: front ramp left disposal"
            },
            /* Servo 0-10 */
            {
                .positions = {
                    1700,   /* Right */
                    1525,   /* Left  */
                    1600,   /* Horizontal */
                },
                .default_position = 2,
                .default_speed = 0,
                .name = "S0-10: front ramp rotation"
            },
            /* Servo 0-11 */
            {
                .positions = {
                    1250,   /* Open */
                    1875,   /* Close */
                    1500,   /* Default */
                },
                .default_position = 1,
                .default_speed = 0,
                .name = "S0-11: front ramp right disposal"
            },
            /* Servo 0-12 */
            {
                .positions = {
                    1900,    /* Open */
                    950,   /* Close */
                    1500,   /* Default */
                },
                .default_position = 0,
                .default_speed = 0,
                .name = "S0-12: front ramp puck block"
            },
        },
    },
    {   /* SD12 ID1 */
        .i2c_dev_id = 0,
        .i2c_address = (0xC4 >> 1),
        .i2c_speed_khz = I2C_SPEED_FAST,

        .servos_nb = 12,
        .servos = {
            /* Servo 1-1 */
            {
                .positions = {
                    1500, /* */
                    1500, /* */
                    1500, /* */
                },
                .default_position = 2,
                .default_speed = 0,
                .name = "S1-1: "
            },
            /* Servo 1-2 */
            {
                .positions = {
                    1500, /* */
                    1500, /* */
                    1500, /* */
                },
                .default_position = 2,
                .default_speed = 0,
                .name = "S1-2: "
            },
            /* Servo 1-3 */
            {
                .positions = {
                    1000, /* Open */
                    1850, /* Close */
                    1500, /* Default */
                },
                .default_position = 0,
                .default_speed = 0,
                .name = "S1-3: back ramp puck block"
            },
            /* Servo 1-4 */
            {
                .positions = {
                    1275,   /* Open */
                    1875,   /* Close */
                    1500,   /* Default */
                },
                .default_position = 1,
                .default_speed = 0,
                .name = "S1-4: back ramp left disposal"
            },
            /* Servo 1-5 */
            {
                .positions = {
                    1750,   /* Open */
                    1075,   /* Close */
                    1500,   /* Default */
                },
                .default_position = 1,
                .default_speed = 0,
                .name = "S1-5: back ramp right disposal"
            },
            /* Servo 1-6 */
            {
                .positions = {
                    1650,   /* Left position */
                    1350,   /* Right position */
                    1500,   /* Horizontal */
                },
                .default_position = 2,
                .default_speed = 0,
                .name = "S1-6: back ramp rotation"
            },
            /* Servo 1-7 */
            {
                .positions = {
                    1000,   /* Top position */
                    1800,   /* Bottom position */
                    1500,   /* Default */
                },
                .default_position = 1,
                .default_speed = 0,
                .name = "S1-7: back right elevator"
            },
            /* Servo 1-8 */
            {
                .positions = {
                    1200,   /* Top position */
                    1975,   /* Bottom position */
                    1500,   /* Default */
                },
                .default_position = 1,
                .default_speed = 0,
                .name = "S1-8: back central elevator"
            },
            /* Servo 1-9 */
            {
                .positions = {
                    2125,   /* Top position */
                    1100,   /* Bottom position */
                    1500,   /* Default */
                },
                .default_position = 1,
                .default_speed = 0,
                .name = "S1-9: back left elevator"
            },
            /* Servo 1-10 */
            {
                .positions = {
                    750,    /* Ramp disposal */
                    2500,   /* Puck prehension */
                    1500,   /* Neutral */
                },
                .default_position = PF_SERVO_STATE_CUP_HOLD,
                .default_speed = 0,
                .name = "S1-10: back right suction cup"
            },
            /* Servo 1-11 */
            {
                .positions = {
                    2225,   /* Ramp disposal */
                    825,   /* Puck prehension */
                    1575,   /* Neutral */
                },
                .default_position = PF_SERVO_STATE_CUP_HOLD,
                .default_speed = 0,
                .name = "S1-11: back central suction cup"
            },
            /* Servo 1-12 */
            {
                .positions = {
                    2200,   /* Ramp disposal */
                    725,   /* Puck prehension */
                    1500,   /* Neutral */
                },
                .default_position = PF_SERVO_STATE_CUP_HOLD,
                .default_speed = 0,
                .name = "S1-12: back left suction cup"
            },
        },
    }
};

#define SD21_NUMOF     (sizeof(sd21_config) / sizeof(sd21_config[0]))

void pf_front_cup_take(void);
void pf_front_cup_hold(void);

#endif /* PLATFORM_H_ */
