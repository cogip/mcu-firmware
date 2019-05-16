/* System includes */
#include <thread.h>

/* RIOT includes */
#include "log.h"
#include "shell.h"
#include "xtimer.h"

/* Project includes */
#include "planner.h"
#include "platform.h"
#include "platform-common.h"

/* Controller */

static ctrl_quadpid_t ctrl_quadpid =
{
    .conf = &ctrl_quadpid_conf,
    .pf_conf = &ctrl_pf_quadpid_conf,
    .quadpid_params = ctrl_quadpid_params,
};

/* Thread stacks */
char controller_thread_stack[THREAD_STACKSIZE_LARGE];
char planner_thread_stack[THREAD_STACKSIZE_LARGE];
char start_shell_thread_stack[THREAD_STACKSIZE_LARGE];

/* Shell command array */
static shell_command_t shell_commands[NB_SHELL_COMMANDS];

void pf_add_shell_command(shell_command_t *command)
{
    static uint8_t command_id = 0;

    assert(command_id < NB_SHELL_COMMANDS);

    shell_commands[command_id++] = *command;
}

ctrl_quadpid_t* pf_get_quadpid_ctrl(void)
{
    return &ctrl_quadpid;
}

path_t *pf_get_path(void)
{
    return &robot_path;
}

void pf_ctrl_pre_running_cb(pose_t *robot_pose, polar_t* robot_speed, polar_t *motor_command)
{
    (void)motor_command;

    /* catch speed */
    encoder_read(robot_speed);

    /* convert to position */
    odometry_update(robot_pose, robot_speed, SEGMENT);
}

void pf_ctrl_post_stop_cb(pose_t *robot_pose, polar_t* robot_speed, polar_t *motor_command)
{
    (void)robot_pose;
    (void)robot_speed;

    /* Set distance and angle command to 0 to stop the robot*/
    motor_command->distance = 0;
    motor_command->angle = 0;

    /* Send command to motors */
    motor_drive(motor_command);
}

void pf_ctrl_post_running_cb(pose_t *robot_pose, polar_t* robot_speed, polar_t *motor_command)
{
    (void)robot_pose;
    (void)robot_speed;

    /* Send command to motors */
    motor_drive(motor_command);
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

void motor_drive(polar_t *command)
{
    int16_t right_command = (int16_t) (command->distance + command->angle);
    int16_t left_command = (int16_t) (command->distance - command->angle);

    motor_set(0, HBRIDGE_MOTOR_LEFT, left_command);
    motor_set(0, HBRIDGE_MOTOR_RIGHT, right_command);
}

void *task_start_shell(void *arg)
{
    int* start_shell = (int*)arg;

    /* Wait for Enter to be pressed */
    getchar();
    /* Set a flag and return once done */
    *start_shell = TRUE;

    puts("Entering calibration mode...");

    return 0;
}

void pf_init_tasks(void)
{
    static int start_shell = FALSE;
    int countdown = PF_START_COUNTDOWN;

    ctrl_t* controller = (ctrl_t*)&ctrl_quadpid;

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

    /* Create controller thread */
    thread_create(controller_thread_stack,
                  sizeof(controller_thread_stack),
                  0, 0,
                  task_ctrl_update,
                  (void*)controller,
                  "motion_ctrl");
    /* Create planner thread */
    thread_create(planner_thread_stack,
                  sizeof(planner_thread_stack),
                  THREAD_PRIORITY_MAIN - 2, 0,
                  task_planner,
                  (void*)controller,
                  "game_planner");

    /* If Enter was pressed, start shell */
    if (start_shell) {
        /* Define buffer to be used by the shell */
        char line_buf[SHELL_DEFAULT_BUFSIZE];
        /* Add end NULL entry to shell_command */
        shell_command_t null_command = { NULL, NULL, NULL };
        pf_add_shell_command(&null_command);
        /* Start shell */
        LOG_DEBUG("platform: Start shell\n");
        shell_run(shell_commands, line_buf, SHELL_DEFAULT_BUFSIZE);
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
        planner_start((ctrl_t*)controller);
    }
}
