/* System includes */
#include <thread.h>
#include <stdlib.h>

/* RIOT includes */
#define ENABLE_DEBUG        (0)
#include "debug.h"
#include "log.h"
#include "shell.h"
#include "xtimer.h"

/* Project includes */
#include "avoidance.h"
#include "obstacle.h"
#include "planner.h"
#include "platform.h"

#ifdef CALIBRATION
#include "calibration/calib_pca9548.h"
#include "calibration/calib_planner.h"
#include "calibration/calib_quadpid.h"
#endif /* CALIBRATION */

/* Controller */
static ctrl_quadpid_t ctrl_quadpid =
{
    .conf = &ctrl_quadpid_conf,
    .pf_conf = &ctrl_pf_quadpid_conf,
};

/* Thread stacks */
char controller_thread_stack[THREAD_STACKSIZE_LARGE];
char countdown_thread_stack[THREAD_STACKSIZE_DEFAULT];
char planner_thread_stack[THREAD_STACKSIZE_LARGE];
char start_shell_thread_stack[THREAD_STACKSIZE_LARGE];
char radio_thread_stack[THREAD_STACKSIZE_DEFAULT];

/* Shell command array */
static shell_command_linked_t current_shell_commands;

shell_command_linked_t pf_shell_commands;

void pf_push_shell_commands(shell_command_linked_t *shell_commands) {
    shell_commands->previous = current_shell_commands.current;
    memcpy(&current_shell_commands, shell_commands, sizeof(shell_command_linked_t));
}

void pf_pop_shell_commands(void) {
    if(current_shell_commands.previous) {
        memcpy(&current_shell_commands, current_shell_commands.previous, sizeof(shell_command_linked_t));
    }
}

void pf_init_shell_commands(shell_command_linked_t *shell_commands) {
    shell_commands->current = shell_commands;
    shell_commands->previous = NULL;

    for(uint8_t i = 0 ; i < NB_SHELL_COMMANDS ; i++) {
        shell_commands->shell_commands[i] = (shell_command_t){ NULL, NULL, NULL };
    }
    shell_commands->shell_commands[0] = cmd_help_json;
}

void pf_add_shell_command(shell_command_linked_t *shell_commands, shell_command_t *command)
{
    uint8_t command_id = 0;

    shell_command_t * entry = shell_commands->shell_commands;
    for (; entry->name != NULL; entry++, command_id++) {}

    assert(command_id < NB_SHELL_COMMANDS);

    shell_commands->shell_commands[command_id++] = *command;
}

int pf_display_json_help(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    puts("{\"cmd\": \"pf_display_json_help()\"}");
    shell_command_t * entry = (shell_command_t*)&current_shell_commands;
    for (; entry->name != NULL; entry++) {
        printf(
            "{\"data\": {\"name\": \"%s\", \"desc\": \"%s\"}}\n",
            entry->name,
            entry->desc
        );
    }
    puts("{\"result\": \"SUCCESS\"}\n");
    return EXIT_SUCCESS;
}

int pf_exit_shell(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    pf_pop_shell_commands();
    return EXIT_SUCCESS;
}

shell_command_t cmd_exit_shell = {
    "exit", "Exit planner calibration",
    pf_exit_shell
};

shell_command_t cmd_help_json = {
    "help_json", "Display available commands in JSON format",
    pf_display_json_help
};

void pf_init_quadpid_params(ctrl_quadpid_parameters_t ctrl_quadpid_params)
{
    ctrl_quadpid.quadpid_params = ctrl_quadpid_params;
}

inline ctrl_quadpid_t* pf_get_quadpid_ctrl(void)
{
    return &ctrl_quadpid;
}

inline ctrl_t* pf_get_ctrl(void)
{
    return (ctrl_t *)&ctrl_quadpid;
}

inline path_t *pf_get_path(void)
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

    ctrl_t *ctrl = pf_get_ctrl();

    if (ctrl_get_mode(ctrl) != CTRL_MODE_STOP)
        DEBUG("@robot@,%u,%"PRIu32",@pose_current@,%.2f,%.2f,%.2f\n",
                    ROBOT_ID,
                    ctrl->control.current_cycle,
                    robot_pose->x,
                    robot_pose->y,
                    robot_pose->O);
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

    ctrl_t *ctrl = pf_get_ctrl();

    /* Only log info when controller is in interesting enough mode */
    if (ctrl_get_mode(ctrl) != CTRL_MODE_STOP && ctrl_get_mode(ctrl) != CTRL_MODE_IDLE) {
        DEBUG("@robot@,%u,%"PRIu32",@qdec_speed@,%"PRIi32",%"PRIi32"\n",
            ROBOT_ID, ctrl->control.current_cycle, left_speed, right_speed);

        DEBUG("@robot@,%u,%"PRIu32",@speed_current@,%.2f,%.2f\n",
            ROBOT_ID,
            ctrl->control.current_cycle,
            robot_speed->distance,
            robot_speed->angle);
    }

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

    ctrl_t *ctrl = pf_get_ctrl();

    /* Only log info when controller is in interesting enough mode */
    if (ctrl_get_mode(ctrl) != CTRL_MODE_STOP && ctrl_get_mode(ctrl) != CTRL_MODE_IDLE) {
        DEBUG("@robot@,%u,%"PRIu32",@speed_set@,%.4f,%.4f\n",
                    ROBOT_ID,
                    ctrl->control.current_cycle,
                    command->distance,
                    command->angle);

        DEBUG("@robot@,%u,%"PRIu32",@motor_set@,%"PRIi16",%"PRIi16"\n",
              ROBOT_ID, ctrl->control.current_cycle, left_command, right_command);
    }

    motor_set(MOTOR_DRIVER_DEV(0), HBRIDGE_MOTOR_LEFT, left_command);
    motor_set(MOTOR_DRIVER_DEV(0), HBRIDGE_MOTOR_RIGHT, right_command);
}

int pf_is_game_launched(void)
{
    /* Starter switch */
    return !gpio_read(GPIO_STARTER);
}

void pf_vl53l0x_reset(void)
{
    for (vl53l0x_t dev = 0; dev < VL53L0X_NUMOF; dev++) {
        pca9548_set_current_channel(PCA9548_SENSORS, vl53l0x_channel[dev]);
        if (vl53l0x_reset_dev(dev) != 0)
            DEBUG("ERROR: Sensor %u reset failed !!!\n", dev);
    }
}

void pf_vl53l0x_init(void)
{
    for (vl53l0x_t dev = 0; dev < VL53L0X_NUMOF; dev++) {
        pca9548_set_current_channel(PCA9548_SENSORS, vl53l0x_channel[dev]);
        if (vl53l0x_init_dev(dev) != 0)
            DEBUG("ERROR: Sensor %u init failed !!!\n", dev);
    }
}

int pf_read_sensors(void)
{
    int obstacle_found = 0;
    int res = 0;

    ctrl_t* ctrl = (ctrl_t*)pf_get_quadpid_ctrl();

    reset_dyn_polygons();

    if (((ctrl_quadpid_t *)ctrl)->quadpid_params.regul == CTRL_REGUL_POSE_PRE_ANGL) {
        return 0;
    }

    for (vl53l0x_t dev = 0; dev < VL53L0X_NUMOF; dev++) {

        uint16_t measure;
        irq_disable();

        pca9548_set_current_channel(PCA9548_SENSORS, vl53l0x_channel[dev]);
        measure = vl53l0x_continuous_ranging_get_measure(dev);

        irq_enable();
        DEBUG("Measure sensor %u: %u\n\n", dev, measure);

        if ((measure > OBSTACLE_DETECTION_MINIMUM_TRESHOLD)
                && (measure < OBSTACLE_DETECTION_MAXIMUM_TRESHOLD)) {
            const pf_sensor_t* sensor = &pf_sensors[dev];
            pose_t robot_pose = *ctrl_get_pose_current(ctrl);

            res = add_dyn_obstacle(dev, &robot_pose, sensor->angle_offset, sensor->distance_offset, (double)measure);

            if (!res) {
                obstacle_found = 1;
            }
        }

    }

    return obstacle_found;
}

void pf_calib_read_sensors(pca9548_t dev)
{
    vl53l0x_t sensor = 0;
    uint8_t channel = pca9548_get_current_channel(dev);
    for (sensor = 0; sensor < VL53L0X_NUMOF; sensor++) {
        if (vl53l0x_channel[sensor] == channel)
            break;
    }

    if (sensor < VL53L0X_NUMOF) {
        uint16_t measure = vl53l0x_continuous_ranging_get_measure(dev);

        printf("Measure sensor %u: %u\n\n", sensor, measure);
    }
    else {
        printf("No sensor for this channel %u !\n\n", sensor);
    }
}


int pf_is_camp_left(void)
{
    /* Color switch for coords translations */
    return !gpio_read(GPIO_CAMP);
}


static void *pf_task_countdown(void *arg)
{
    (void)arg;
    static int countdown = GAME_DURATION_SEC;

    ctrl_t* controller = (ctrl_t*)&ctrl_quadpid;

    for (;;) {
        xtimer_ticks32_t loop_start_time = xtimer_now();
        if (countdown < 0) {
            pln_stop(controller);
        }
        else {
            DEBUG("                                      GAME TIME: %d\n",
                countdown);
            countdown--;
        }
        xtimer_periodic_wakeup(&loop_start_time, US_PER_SEC);
    }

    return NULL;
}

#ifdef CALIBRATION
static void *pf_task_start_shell(void *arg)
{
    int* start_shell = (int*)arg;

    /* Wait for Enter to be pressed */
    getchar();
    /* Set a flag and return once done */
    *start_shell = TRUE;

    puts("Entering calibration mode...");

    return NULL;
}
#endif  /* CALIBRATION */

void pf_init_tasks(void)
{
    static int start_shell = FALSE;

    ctrl_t* controller = (ctrl_t*)&ctrl_quadpid;

#ifdef CALIBRATION
    int countdown = PF_START_COUNTDOWN;

    puts("Built-in calibration mode is ACTIVATED");

    /* Create thread that up a flag on key pressed to start a shell instead of
       planner below */
    kernel_pid_t start_shell_pid = thread_create(start_shell_thread_stack,
                  sizeof(start_shell_thread_stack),
                  THREAD_PRIORITY_MAIN + 1, 0,
                  pf_task_start_shell, &start_shell, "shell");

    puts("Press Enter to enter calibration mode...");

    /* Wait for Enter key pressed or countdown */
    while ((!start_shell) && (countdown > 0)) {
        xtimer_ticks32_t loop_start_time = xtimer_now();
        printf("%d seconds left...\n", countdown);
        countdown--;
        xtimer_periodic_wakeup(&loop_start_time, US_PER_SEC);
    }
#endif  /* CALIBRATION */

    /* Create controller thread */
    thread_create(controller_thread_stack,
                  sizeof(controller_thread_stack),
                  THREAD_PRIORITY_MAIN - 4, 0,
                  task_ctrl_update,
                  (void*)controller,
                  "motion control");
    /* Create planner thread */
    thread_create(planner_thread_stack,
                  sizeof(planner_thread_stack),
                  THREAD_PRIORITY_MAIN - 2, 0,
                  task_planner,
                  NULL,
                  "planner");

    /* If Enter was pressed, start shell */
    if (start_shell) {
        /* Define buffer to be used by the shell */
        char line_buf[SHELL_DEFAULT_BUFSIZE];

        pf_push_shell_commands(&pf_shell_commands);

        /* Start shell */
        DEBUG("platform: Start shell\n");
        shell_run((shell_command_t*)&current_shell_commands, line_buf, SHELL_DEFAULT_BUFSIZE);
    }
    /* Else start game */
    else {
        /* Stop useless task_start_shell thread still running */
#ifdef CALIBRATION
        thread_t* start_shell_thread = (thread_t*)thread_get(start_shell_pid);
        if (start_shell_thread) {
            sched_set_status(start_shell_thread, STATUS_STOPPED);
        }
#endif  /* CALIBRATION */

        /* Wait for start switch */
        while(!pf_is_game_launched());

        /* Debug indicator to track the non starting state */
        gpio_set(GPIO_DEBUG_LED);

        /* Create countdown thread */
        thread_create(countdown_thread_stack,
                sizeof(countdown_thread_stack),
                THREAD_PRIORITY_MAIN - 3, 0,
                pf_task_countdown,
                NULL,
                "countdown");

        /* Start game */
        DEBUG("platform: Start game\n");
        pln_start((ctrl_t*)controller);
    }
}

void pf_init(void)
{
    pf_init_shell_commands(&pf_shell_commands);

    motor_driver_init(MOTOR_DRIVER_DEV(0));

    /* Setup qdec periphereal */
    int error = qdec_init(QDEC_DEV(HBRIDGE_MOTOR_LEFT), QDEC_MODE, NULL, NULL);
    if (error) {
        printf("QDEC %u not initialized, error=%d !!!\n", HBRIDGE_MOTOR_LEFT, error);
    }
    error = qdec_init(QDEC_DEV(HBRIDGE_MOTOR_RIGHT), QDEC_MODE, NULL, NULL);
    if (error) {
        printf("QDEC %u not initialized, error=%d !!!\n", HBRIDGE_MOTOR_RIGHT, error);
    }

    /* Init odometry */
    odometry_setup(WHEELS_DISTANCE / PULSE_PER_MM);

    /* Init starter and camp selection GPIOs */
    if (gpio_init(GPIO_CAMP, GPIO_IN) == 0) {
        puts("WARNING: GPIO_CAMP not initialized !");
    }
    if (gpio_init(GPIO_STARTER, GPIO_IN_PU) == 0) {
        puts("WARNING: GPIO_STARTER not initialized !");
    }

    /* Debug LED */
    if (gpio_init(GPIO_DEBUG_LED, GPIO_OUT) == 0) {
        puts("WARNING: GPIO_DEBUG_LED not initialized !");
    }

    gpio_clear(GPIO_DEBUG_LED);

    pca9548_init();

    for (vl53l0x_t dev = 0; dev < VL53L0X_NUMOF; dev++) {
        pca9548_set_current_channel(PCA9548_SENSORS, vl53l0x_channel[dev]);
        if (vl53l0x_init_dev(dev) != 0)
            printf("ERROR: Sensor %u init failed !!!\n", dev);
    }

    ctrl_set_anti_blocking_on(pf_get_ctrl(), TRUE);

    /* Get platform path */
    path_t* path = pf_get_path();
    if (!path) {
        printf("machine has no path\n");
    }

    /* mirror the points in place if selected camp is left */
    if (pf_is_camp_left()) {
        path_horizontal_mirror_all_pos(path);
    }

#ifdef CALIBRATION
    ctrl_quadpid_calib_init();
    pca9548_calib_init();
    pln_calib_init();
#endif /* CALIBRATION */
}
