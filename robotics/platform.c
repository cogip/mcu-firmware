#include <stdio.h>
#include <stdarg.h>

#include "platform.h"
#include "planner.h"
#include <periph/qdec.h>
#include <motor_driver.h>
#include <periph/adc.h>
#include "ctrl/quadpid.h"

#include <thread.h>

#include "xtimer.h"

char controller_thread_stack[THREAD_STACKSIZE_DEFAULT];
char planner_thread_stack[THREAD_STACKSIZE_DEFAULT];

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
    .conf = ctrl_quadpid_conf,
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
    .min_distance_for_angular_switch = 3 /* mm */,
    .min_angle_for_pose_reached = 2 /* °deg */,
    .regul = CTRL_REGUL_POSE_DIST,
};

path_t *pf_get_path(void)
{
    return &robot_path;
}

uint8_t pf_is_game_launched(void)
{
    /* Starter switch */
    return 1;
}

uint8_t pf_is_camp_left(void)
{
    /* Color switch for coords translations */
    return 0;
}

void pf_setup(void)
{
    motor_driver_init(0);

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
    odometry_setup(WHEELS_DISTANCE / PULSE_PER_MM);
}

void ctrl_state_stop_cb(pose_t *robot_pose, polar_t* robot_speed, polar_t *motor_command)
{
    (void)robot_pose;
    (void)robot_speed;
    /* final position */
    motor_command->distance = 0;
    motor_command->angle = 0;
}

void ctrl_state_idle_cb(pose_t *robot_pose, polar_t* robot_speed, polar_t *motor_command)
{
    (void)motor_command;
    (void)robot_pose;
    (void)robot_speed;
}

void ctrl_state_ingame_cb(pose_t *robot_pose, polar_t* robot_speed, polar_t *motor_command)
{
    (void)motor_command;

    /* catch speed */
    encoder_read(robot_speed);

    /* convert to position */
    odometry_update(robot_pose, robot_speed, SEGMENT);

    ctrl_set_pose_current((ctrl_t*)&controller, robot_pose);
    ctrl_set_speed_current((ctrl_t*)&controller, robot_speed);
}

int encoder_read(polar_t *robot_speed)
{
    int32_t left_speed = qdec_read_and_reset(HBRIDGE_MOTOR_LEFT) * QDEC_LEFT_POLARITY;
    int32_t right_speed = qdec_read_and_reset(HBRIDGE_MOTOR_RIGHT) * QDEC_RIGHT_POLARITY;

    /* update speed */
    robot_speed->distance = ((right_speed + left_speed) / 2.0) / PULSE_PER_MM;
    robot_speed->angle = (right_speed - left_speed) / PULSE_PER_DEGREE;

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

void pf_tasks_init(void)
{
    thread_create(controller_thread_stack, sizeof(controller_thread_stack),
                  0, 0,
                  task_ctrl_update, &controller, "motion_ctrl");
    thread_create(planner_thread_stack, sizeof(planner_thread_stack),
                  5, 0,
                  task_planner, &controller, "game_planner");
    planner_start_game((ctrl_t*)&controller);
}
