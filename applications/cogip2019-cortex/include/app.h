#ifndef PLATFORM_H_
#define PLATFORM_H_

#define SD21_SERVO_NUMOF        12
#define SD21_SERVO_POS_NUMOF    3
#define SD21_SERVO_POS_MIN      500
#define SD21_SERVO_POS_MAX      2500

/* Project includes */
#include "board.h"
#include "platform.h"
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
#define PULSE_PER_MM        10.624
/* WHEELS_DISTANCE = WHEELS_DISTANCE_MM * PULSE_PER_MM */
#define WHEELS_DISTANCE     2974.72
/* WHEELS_DISTANCE*2*pi pulses for 360 deg. Thus 51.76 pulses per deg */
#define PULSE_PER_DEGREE    51.91


#define USART_CONSOLE   USARTC0

#define PCA9548_CALIB_CB(x) pf_calib_read_sensors(x)

/**********/
/* Servos */
/**********/

/* Suction cups */
#define PF_SERVO_FL_CUP             0, 0
#define PF_SERVO_FC_CUP             0, 1
#define PF_SERVO_FR_CUP             0, 2

#define PF_SERVO_BL_CUP             1, 11
#define PF_SERVO_BC_CUP             1, 10
#define PF_SERVO_BR_CUP             1, 9

/* Elevators */
#define PF_SERVO_FC_ELEVATOR        0, 4
#define PF_SERVO_FL_ELEVATOR        0, 5
#define PF_SERVO_FR_ELEVATOR        0, 7

#define PF_SERVO_BC_ELEVATOR        1, 7
#define PF_SERVO_BL_ELEVATOR        1, 8
#define PF_SERVO_BR_ELEVATOR        1, 6

/* Fork */
#define PF_SERVO_FORK               1, 2

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
#define PF_SERVO_STATE_CUP_RAMP     0
#define PF_SERVO_STATE_CUP_TAKE     1
#define PF_SERVO_STATE_CUP_HOLD     2

/* Elevators */
#define PF_SERVO_STATE_ELEVATOR_TOP     0
#define PF_SERVO_STATE_ELEVATOR_BOTTOM  1
#define PF_SERVO_STATE_ELEVATOR_GOLDEN  2
//TODO: check in struct below. #define PF_SERVO_STATE_ELEVATOR_FORK    XXX

/* Fork */
#define PF_SERVO_STATE_FORK_TOP         0
#define PF_SERVO_STATE_FORK_BOTTOM      1

/* Ramps */
#define PF_SERVO_STATE_RAMP_OPEN        0
#define PF_SERVO_STATE_RAMP_CLOSE       1
#define PF_SERVO_STATE_RAMP_RIGHT       0
#define PF_SERVO_STATE_RAMP_LEFT        1
#define PF_SERVO_STATE_RAMP_HORIZ       2

/* Arms */
#define PF_SERVO_STATE_ARM_OPEN         0
#define PF_SERVO_STATE_ARM_CLOSE        1

/*********/
/* Radio */
/*********/

#ifndef CC110X_PARAM_SPI
#define CC110X_PARAM_SPI            SPI_DEV(0)
#endif

#ifndef CC110X_PARAM_CS
#define CC110X_PARAM_CS             GPIO_PIN(PORT_A, 1)
#endif

#ifndef CC110X_PARAM_GDO0
#define CC110X_PARAM_GDO0           GPIO_PIN(PORT_C, 4)
#endif

/* Note: GDO1 is muxed with MISO pin */
#ifndef CC110X_PARAM_GDO1
#define CC110X_PARAM_GDO1           GPIO_PIN(PORT_C, 2)
#endif

#ifndef CC110X_PARAM_GDO2
#define CC110X_PARAM_GDO2           GPIO_PIN(PORT_A, 0)
#endif

#ifndef CC110X_PARAMS
#define CC110X_PARAMS               { \
                                        .spi  = CC110X_PARAM_SPI,  \
                                        .cs   = CC110X_PARAM_CS,   \
                                        .gdo0 = CC110X_PARAM_GDO0, \
                                        .gdo1 = CC110X_PARAM_GDO1, \
                                        .gdo2 = CC110X_PARAM_GDO2, \
                                    }
#endif

typedef struct {
    double angle_offset;
    double distance_offset;
} pf_sensor_t;

typedef struct {
    uint8_t nb_puck_front_ramp;
    uint8_t nb_puck_back_ramp;
    uint8_t front_ramp_blocked;
    uint8_t back_ramp_blocked;
    uint8_t front_arms_opened;
    uint8_t goldenium_opened;
    uint8_t goldenium_taken;
    uint8_t red_puck_on_hold_front;
    uint8_t red_puck_on_hold_back;
    uint8_t any_pump_on;
    uint8_t front_fork_occupied;
} pf_actions_context_t;

static const pf_sensor_t pf_sensors[VL53L0X_NUMOF] = {
    {
        .angle_offset = 135,
        .distance_offset = ROBOT_MARGIN,
    },
    {
        .angle_offset = 180,
        .distance_offset = ROBOT_MARGIN,
    },
    {
        .angle_offset = -135,
        .distance_offset = ROBOT_MARGIN,
    },
    {
        .angle_offset = -45,
        .distance_offset = ROBOT_MARGIN,
    },
    {
        .angle_offset = 0,
        .distance_offset = ROBOT_MARGIN,
    },
    {
        .angle_offset = 45,
        .distance_offset = ROBOT_MARGIN,
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
        .i2c_speed_khz = I2C_SPEED_NORMAL,

        .servos_nb = 12,
        .servos = {
            /* Servo 0-1 */
            {
                .positions = {
                    750,  /* Ramp Disposal */
                    2325, /* Puck prehension */
                    1500, /* Neutral */
                },
                .default_position = PF_SERVO_STATE_CUP_HOLD,
                .default_speed = 0,
                .name = "S0-1: front left suction cup"
            },
            /* Servo 0-2 */
            {
                .positions = {
                    2500, /* Ramp Disposal */
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
                     950,   /* Top position */
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
                    1025,   /* Top position */
                    2000,   /* Bottom position */
                    1425,   /* Goldenium */
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
                    1600,   /* Goldenium */
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
                    1900,   /* Right */
                    1475,   /* Left  */ // FIXME: réparer la rampe !
                    1675,   /* Horizontal */
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
                    950,    /* Open */
                    1900,   /* Close */
                    950,    /* Default */
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
        .i2c_speed_khz = I2C_SPEED_NORMAL,

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
                    1975, /* Top position */
                    1350, /* Bottom position */
                    1975, /* Default */
                },
                .default_position = 2,
                .default_speed = 0,
                .name = "S1-2: Front fork"
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
                    1350,   /* Right position */
                    1650,   /* Left position */
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
                    1100,   /* Top position */
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
                    2425,   /* Puck prehension */
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

void pf_calib_read_sensors(pca9548_t dev);

#define SD21_NUMOF     (sizeof(sd21_config) / sizeof(sd21_config[0]))

void pf_stop_pumps(void);

void pf_front_cup_take(void);
void pf_front_cup_ramp(void);
void pf_back_cup_take(void);
void pf_back_cup_ramp(void);

void pf_front_ramp_right_drop(void);
void pf_front_ramp_reset(void);
void pf_back_ramp_left_drop(void);
void pf_back_ramp_reset(void);
void pf_back_ramp_left_horiz_for_goldenium(void);
void pf_arms_open(void);
void pf_arms_close(void);
void pf_goldenium_hold(void);
void pf_goldenium_take(void);
void pf_goldenium_drop(void);

#endif /* PLATFORM_H_ */
