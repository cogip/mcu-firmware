#include <stdio.h>
#include <stdarg.h>

#include "platform.h"
#include "planner.h"
#include <periph/qdec.h>
#include <motor_driver.h>
#include "analog_sensor.h"
#include <periph/adc.h>
#include "ctrl/quadpid.h"

#include "actuators/sd21.h"
#if defined(CONFIG_MOTOR_PAP)
#include "actuators/motor_pap.h"
#endif
#include <thread.h>

#include "xtimer.h"

extern analog_sensors_t ana_sensors;

char controller_thread_stack[THREAD_STACKSIZE_DEFAULT];
char planner_thread_stack[THREAD_STACKSIZE_DEFAULT];
char analog_sensors_thread_stack[THREAD_STACKSIZE_DEFAULT];

#if defined(MODULE_CALIBRATION)
#define CALIBRATION_BOOT_DELAY 3

char calib_thread_stack[THREAD_STACKSIZE_DEFAULT];
char calib_wait_thread_stack[THREAD_STACKSIZE_DEFAULT];

typedef enum {
    CALIBRATION_NONE = 0,
    CALIBRATION_READY,
    CALIBRATION_STARTED,
} calibration_state_t;

/* Calibration flag to launch or not calibration thread */
static volatile calibration_state_t calib_flag = 0;
#endif /* MODULE_CALIBRATION */


static ctrl_quadpid_t controller = {
    .common = {
        .allow_reverse = TRUE,
        .current_mode = NULL,
        .modes = {
            {
                .mode_id = CTRL_STATE_STOP,
                .name = "STOP",
                .mode_cb = ctrl_state_stop_cb
            },
            {
                .mode_id = CTRL_STATE_IDLE,
                .name = "IDLE",
                .mode_cb = ctrl_state_idle_cb
            },
            {
                .mode_id = CTRL_STATE_BLOCKED,
                .name = "BLOCKED",
                .mode_cb = ctrl_state_stop_cb
            },
            {
                .mode_id = CTRL_STATE_INGAME,
                .name = "INGAME",
                .mode_cb = ctrl_state_ingame_cb
            },
        },
    },
    .linear_speed_pid = {
        .kp = 15.,
        .ki = 2.,
        .kd = 0.,
    },
    .angular_speed_pid = {
        .kp = 15.,
        .ki = 2.,
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

    //.min_distance_for_angular_switch = 500,
    .min_distance_for_angular_switch = 30,
    .min_angle_for_pose_reached = 100,
    .regul = CTRL_REGUL_POSE_DIST,
};

/* TODO: To activate when included in RIOT */
analog_sensors_t ana_sensors = {
    .sensors_nb = 6,
    .sensor_index = 0,
    .sensors = {
        /* Front: [10...80] cm - GP2Y0A21 - cal done */
        [0] = {
            .adc = 0,

            .pos_str = "F",

            .coeff_volts = 0.022,
            .const_volts = 0.010,
            .const_dist = -5.0,
            .dist_cm_max = 100,

            .dist_robot_offset_cm = 14,
            .angle_robot_offset = 0,
        },
        /* Rear: [10...80] cm - GP2Y0A21 - cal done */
        [1] = {
            .adc = 1,

            .pos_str = "R",

            .coeff_volts = 0.022,
            .const_volts = 0.010,
            .const_dist = -5.0,
            .dist_cm_max = 100,

            .dist_robot_offset_cm = 14,
            .angle_robot_offset = 180,
        },
        /* Front Side left: [4...30] cm - GP2YD120X - cal done */
        [2] = {
            .adc = 2,

            .pos_str = "FL",

            .coeff_volts = 0.052,
            .const_volts = 0.007,
            .const_dist = 0,
            .dist_cm_max = 40,

            .dist_robot_offset_cm = 16,
            .angle_robot_offset = 45,
        },
        /* Front Side right: [4...30] cm - GP2YD120X - cal done */
        [3] = {
            .adc = 3,

            .pos_str = "FR",

            .coeff_volts = 0.052,
            .const_volts = 0.007,
            .const_dist = 0,
            .dist_cm_max = 40,

            .dist_robot_offset_cm = 16,
            .angle_robot_offset = 315,
        },
        /* Rear Side left: [4...30] cm - GP2YD120X - cal done */
        [4] = {
            .adc = 4,

            .pos_str = "RL",

            .coeff_volts = 0.052,
            .const_volts = 0.007,
            .const_dist = 0,
            .dist_cm_max = 40,

            .dist_robot_offset_cm = 16,
            .angle_robot_offset = 135,
        },
        /* Rear Side right: [4...30] cm - GP2YD120X - cal done */
        [5] = {
            .adc = 5,

            .pos_str = "RR",

            .coeff_volts = 0.052,
            .const_volts = 0.007,
            .const_dist = 0,
            .dist_cm_max = 40,

            .dist_robot_offset_cm = 16,
            .angle_robot_offset = 225,
        },
    }
};

#if defined(CONFIG_SD21)
sd21_t sd21 = {
    .bus_id = 0,
    .twi_speed_khz = I2C_SPEED_FAST,

    .servos_nb = SERVO_COUNT,
    .servos = {
        [SERVO_ID_VALVE_LAUNCHER] = {
            .value_init = 1350,
            .value_open = 2400,
            .value_close = 1350,
        },

        [SERVO_ID_VALVE_RECYCLER] = {
            .value_init = 1050,
            .value_open = 1925,
            .value_close = 1050,
        },

        [SERVO_ID_RECYCLER] = {
            .value_init = 1750,
            .value_open = 1750,
            .value_close = 1200,
        },

        [SERVO_ID_BEE_L] = {
            .value_init = 2125,
            .value_open = 1225,
            .value_close = 2125,
        },

        [SERVO_ID_BEE_R] = {
            .value_init = 875,
            .value_open = 1700,
            .value_close = 875,
        },

    },
};
#endif /* CONFIG_SD21 */

/* TCE0 ClkIn == ClkPer / 8 == 4000 KHz */
/* Counter set to 200 for 20KHz output */
#define TC_MOTOR_PRESCALER      TC_CLKSEL_DIV8_gc
#define TC_MOTOR_PER_VALUE      200

/* This global object contains all numerical logs references (vectors, etc.) */
datalog_t datalog;

static void pf_post_ctrl_loop_func(void)
{
    //analog_sensor_refresh_all(&ana_sensors);
}

inline func_cb_t pf_get_ctrl_loop_pre_pfn(void)
{
    return NULL;
}

inline func_cb_t pf_get_ctrl_loop_post_pfn(void)
{
    return pf_post_ctrl_loop_func;
}

inline func_cb_t pf_get_end_of_game_pfn(void)
{
    return NULL;
}

path_t *pf_get_path(void)
{
    return &robot_path;
}

uint8_t pf_is_game_launched(void)
{
    /* Starter switch */
#if defined(CONFIG_USE_STARTER) && !defined(BOARD_NATIVE)
    /* read 4 when the bar is NOT yet removed */
    return gpio_read(GPIO_PIN(PORT_B, 2)) ? 0 : 1;
#else
    return 1;
#endif
}

uint8_t pf_is_camp_left(void)
{
    /* Color switch for coords translations */
    gpio_init(GPIO_PIN(PORT_B, 10), GPIO_IN);

    xtimer_usleep(100 * US_PER_MS); // Wait 100ms (debounce)

    return gpio_read(GPIO_PIN(PORT_B, 10)) ? 1 : 0;
}

void pf_setup(void)
{
#if F_CPU == 32000000UL
    clksys_intrc_32MHz_setup();
#endif

#ifdef CONFIG_ENABLE_LOGGING
    /* setup logs through usart communication */
    console_init(&usartc0_console);
#endif

    analog_sensor_setup(&ana_sensors);

#if defined(CONFIG_SD21)
    /* setup TWI communication with SD21 */
    sd21_setup(&sd21);
#endif /* CONFIG_SD21 */

    //action_setup(); /* TODO: commenter pour debug */

    motor_driver_init(0);
#if defined(CONFIG_MOTOR_PAP)
    motor_pap_init();
#endif

    /* Starter, orig 1 & 2, color */
    gpio_init(GPIO_PIN(PORT_A, 0), GPIO_IN_PU);
    gpio_init(GPIO_PIN(PORT_A, 1), GPIO_IN_PU);
    gpio_init(GPIO_PIN(PORT_B, 2), GPIO_IN_PU);
    gpio_init(GPIO_PIN(PORT_B, 10), GPIO_IN);

    action_init();

    /* setup qdec */
    int error = qdec_init(QDEC_DEV(HBRIDGE_MOTOR_LEFT), QDEC_MODE, NULL, NULL);
    if (error) {
        printf("QDEC %u not initialized, error=%d !!!\n", HBRIDGE_MOTOR_LEFT, error);
    }
    error = qdec_init(QDEC_DEV(HBRIDGE_MOTOR_RIGHT), QDEC_MODE, NULL, NULL);
    if (error) {
        printf("QDEC %u not initialized, error=%d !!!\n", HBRIDGE_MOTOR_RIGHT, error);
    }

    /* controller setup */
    odometry_setup(WHEELS_DISTANCE);

#if defined(__AVR__)
    /* Programmable Multilevel Interrupt Controller */
    PMIC.CTRL |= PMIC_LOLVLEN_bm; /* Low-level Interrupt Enable */

    /* global interrupt enable */
    sei();
#endif

    log_vect_init(&datalog, NULL, /*400,*/
                  COL_INT32, "left_speed",
                  COL_INT32, "right_speed",
                  COL_INT16, "left_command",
                  COL_INT16, "right_command",
                  COL_DOUBLE, "robot_speed.distance",
                    //COL_DOUBLE, "robot_speed.angle",
                  COL_DOUBLE, "speed_order.distance",
                    //COL_DOUBLE, "speed_order.angle",
                  COL_END);
}

void ctrl_state_stop_cb(pose_t *robot_pose, polar_t *motor_command)
{
    (void)robot_pose;
    /* final position */
    motor_command->distance = 0;
    motor_command->angle = 0;
}

void ctrl_state_idle_cb(pose_t *robot_pose, polar_t *motor_command)
{
    (void)motor_command;
    (void)robot_pose;
}

void ctrl_state_ingame_cb(pose_t *robot_pose, polar_t *motor_command)
{
    polar_t robot_speed = { 0, 0 };

    /* catch speed */
    encoder_read(&robot_speed);

    /* convert to position */
    odometry_update(robot_pose, &robot_speed, SEGMENT);

    /* convert pulse to degree */
    robot_pose->O /= PULSE_PER_DEGREE;

    /* PID / feedback control */
    *motor_command = ctrl_update(&controller,
                                       robot_pose,
                                       robot_speed);

    /* convert degree to pulse */
    robot_pose->O *= PULSE_PER_DEGREE;
}

int encoder_read(polar_t *robot_speed)
{
    int32_t left_speed = qdec_read_and_reset(HBRIDGE_MOTOR_LEFT) * QDEC_LEFT_POLARITY;
    int32_t right_speed = qdec_read_and_reset(HBRIDGE_MOTOR_RIGHT) * QDEC_RIGHT_POLARITY;

    /* update speed */
    robot_speed->distance = (right_speed + left_speed) / 2.0;
    robot_speed->angle = right_speed - left_speed;

    log_vect_setvalue(&datalog, LOG_IDX_SPEED_L, (void *) &left_speed);
    log_vect_setvalue(&datalog, LOG_IDX_SPEED_R, (void *) &right_speed);

    return 0;
}

void encoder_reset(void)
{
    qdec_read_and_reset(HBRIDGE_MOTOR_LEFT);
    qdec_read_and_reset(HBRIDGE_MOTOR_RIGHT);
}

/* FIXME: put following func elsewhere */
/* Note: requires "LINKFLAGS += -u _scanf_float" */
int custom_scanf(const char *format, ...)
{
    va_list args;

    int c, retval;
    char buffer[100];
    uint8_t idx = 0;

    while ((c = cons_getchar()) != '\r' && idx < 100) {
        printf("%c", c); fflush(stdout);
        buffer[idx++] = c;
    }
    buffer[idx] = '\0';

    va_start(args, format);
    retval = vsscanf(buffer, format, args);
    va_end(args);

    return retval;
}

#if defined(MODULE_CALIBRATION)
/**
 * @brief Wait for calibration to be started
 *
 * @param[in] arg		thread args (not used)
 *
 * @return				0 for success
 */
static void *calib_wait(void *arg)
{
    (void)arg;

    /* Thread period */
    xtimer_ticks32_t loop_start_time = xtimer_now();
    /* start time and current time to check if CALIBRATION_BOOT_DELAY is exceeded */
    uint32_t start_time = xtimer_now_usec();
    uint32_t current_time = xtimer_now_usec();
    /* Counter for display purpose */
    uint8_t sec_count = 0;

    /* Wait until calibration flag is raised in calling thread or delay is exceeded */
    while ((calib_flag == CALIBRATION_NONE)
           && (current_time - start_time < CALIBRATION_BOOT_DELAY * US_PER_SEC)) {
        current_time = xtimer_now_usec();
        xtimer_periodic_wakeup(&loop_start_time, THREAD_PERIOD_INTERVAL);
        if (current_time > sec_count * US_PER_SEC) {
            printf("%u seconds left...\n", CALIBRATION_BOOT_DELAY - sec_count++);
        }
    }

    /* On calibration flag raised, calibration thread is ready */
    if (calib_flag == CALIBRATION_READY) {
        puts("Calibration started !");
        /* Up calibration flag to start calibration task */
        calib_flag = CALIBRATION_STARTED;
    }
    /* else launch game */
    else {
        planner_start_game();
    }

    return 0;
}
#endif /* MODULE_CALIBRATION */

void pf_tasks_init(void)
{
/* FIXME: Launch calibration task */
    thread_create(controller_thread_stack, sizeof(controller_thread_stack),
                  0, 0,
                  task_ctrl_update, &controller, "motion_ctrl");
    thread_create(planner_thread_stack, sizeof(planner_thread_stack),
                  5, 0,
                  task_planner, &controller, "game_planner");
    thread_create(analog_sensors_thread_stack, sizeof(analog_sensors_thread_stack),
                  10, 0,
                  task_analog_sensors, (void *)&ana_sensors, "analog_sensors");
#if defined(MODULE_CALIBRATION)
    thread_create(calib_wait_thread_stack, sizeof(calib_wait_thread_stack),
                  0, 0,
                  calib_wait, NULL, "wait_calibration");

    puts("Press a key to enter calibration...");
    cons_getchar();
    /* Calibration is ready */
    calib_flag = CALIBRATION_READY;
    /* Wait for calibration to be started */
    while (calib_flag < CALIBRATION_STARTED) {}
    /* Launch calibration function */
    task_calibration_entry(NULL);
#else
    planner_start_game((ctrl_t*)&controller);
#endif /* MODULE_CALIBRATION */
}
