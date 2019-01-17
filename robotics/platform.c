#include <stdio.h>
#include <stdarg.h>

#include "platform.h"
#include "planner.h"
#include <periph/qdec.h>
#include <motor_driver.h>
#include <periph/adc.h>
#include "ctrl/quadpid.h"

#include <thread.h>

/* RIOT includes */
#include "log.h"
#include "shell.h"
#include "xtimer.h"

char controller_thread_stack[THREAD_STACKSIZE_DEFAULT];
char planner_thread_stack[THREAD_STACKSIZE_DEFAULT];
char start_shell_thread_stack[THREAD_STACKSIZE_DEFAULT];

static ctrl_quadpid_t controller = {
    .common = {
        .allow_reverse = TRUE,
        .current_mode = CTRL_STATE_STOP,
        .ctrl_pre_mode_cb[CTRL_STATE_INGAME] = ctrl_state_ingame_cb,
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

void *task_start_shell(void *arg)
{
    int* start_shell = (int*)arg;

    /* Wait for Enter to be pressed */
    getchar();
    /* Set a flag and return once done */
    *start_shell = TRUE;

    return 0;
}

void pf_tasks_init(void)
{
    static int start_shell = FALSE;
    int countdown = PF_START_COUNTDOWN;

    /* Create controller thread */
    thread_create(controller_thread_stack,
                  sizeof(controller_thread_stack),
                  0, 0,
                  task_ctrl_update, &controller, "motion_ctrl");
    /* Create planner thread */
    thread_create(planner_thread_stack,
                  sizeof(planner_thread_stack),
                  10, 0,
                  task_planner, &controller, "game_planner");
    /* Create thread that up a flag on key pressed to start a shell instead of
       planner below */
    kernel_pid_t start_shell_pid = thread_create(start_shell_thread_stack,
                  sizeof(start_shell_thread_stack),
                  THREAD_PRIORITY_IDLE - 1, 0,
                  task_start_shell, &start_shell, "shell");

    LOG_INFO("Press Enter to enter calibration mode...\n");

    /* Wait for Enter key pressed or countdown */
    while ((!start_shell) && (countdown > 0)) {
        LOG_INFO("%d left...\n", countdown--);
        xtimer_sleep(1);
    }

    /* If Enter was pressed, start shell */
    if (start_shell) {
        /* Define buffer to be used by the shell */
        char line_buf[SHELL_DEFAULT_BUFSIZE];
        /* Start shell */
        LOG_DEBUG("platform: Start shell\n");
        shell_run(NULL, line_buf, SHELL_DEFAULT_BUFSIZE);
    }
    /* Else start game */
    else {
        /* Stop useless task_start_shell thread still running */
        thread_t* start_shell_thread = (thread_t*)thread_get(start_shell_pid);
        if (start_shell_thread) {
            sched_set_status(start_shell_thread, STATUS_STOPPED);
        }

        /* Start game */
        LOG_DEBUG("platform: Start game\n");
        planner_start_game((ctrl_t*)&controller);
    }
}
