/* RIOT includes */
#include "log.h"
#include "xtimer.h"
#include "periph/qdec.h"

/* Project includes */
#include "app.hpp"
#include "board.h"
#include "obstacles/obstacles.hpp"
#include "planners/astar/AstarPlanner.hpp"
#include "platform.hpp"
#include "avoidance.hpp"
#include "tracefd/tracefd.hpp"
#include "path/Path.hpp"

/* Platform includes */
#include "lidar_utils.hpp"
#include "lidar_obstacles.hpp"
#include "trace_utils.hpp"

#ifdef MODULE_SHELL_PLATFORMS
#include "shell_platforms.hpp"
#endif /* MODULE_SHELL_PLATFORMS */

#ifdef MODULE_SHELL_QUADPID
#include "shell_quadpid.hpp"
#endif /* MODULE_SHELL_QUADPID */

#define ENABLE_DEBUG        (0)
#include "debug.h"

/* Controller */
static ctrl_quadpid_t ctrl_quadpid =
{
    .conf = &ctrl_quadpid_conf,
    .pf_conf = &ctrl_pf_quadpid_conf,
    .control = {
        .pose_order = { 0, 0, 0 },
        .pose_current = { 0 ,0, 0 },
        .speed_order{ 0, 0 },
        .speed_current = { 0, 0 },
        .speed_order_cb = 0,
        .pose_reached = 0,
        .pose_intermediate = 0,
        .allow_reverse = 0,
        .anti_blocking_on = 0,
        .blocking_cycles = 0,
        .current_mode = CTRL_MODE_STOP,
        .current_cycle = 0
    },
    .quadpid_params = {
        .linear_speed_pid = { 0, 0, 0, 0, 0 },
        .angular_speed_pid = { 0, 0, 0, 0, 0 },
        .linear_pose_pid = { 0, 0, 0, 0, 0 },
        .angular_pose_pid = { 0, 0, 0, 0, 0 },
        .min_distance_for_angular_switch = 0,
        .min_angle_for_pose_reached = 0,
        .regul = CTRL_REGUL_IDLE
    }
};

/* Planner */
cogip::planners::Planner *planner = nullptr;

/* Thread stacks */
char controller_thread_stack[THREAD_STACKSIZE_LARGE];
char countdown_thread_stack[THREAD_STACKSIZE_DEFAULT];
char planner_start_cancel_thread_stack[THREAD_STACKSIZE_DEFAULT];

static bool trace_on = false;

bool pf_trace_on(void)
{
    return trace_on;
}

void pf_set_trace_mode(bool state)
{
    trace_on = state;
}

void pf_print_state(cogip::tracefd::File &out)
{
    ctrl_t *ctrl = (ctrl_t *)&ctrl_quadpid;

    out.lock();

    out.printf(
        "{"
        "\"mode\":%u,"
        "\"pose_current\":{\"O\":%.3lf,\"x\":%.3lf,\"y\":%.3lf},"
        "\"pose_order\":{\"O\":%.3lf,\"x\":%.3lf,\"y\":%.3lf},"
        "\"cycle\":%" PRIu32 ","
        "\"speed_current\":{\"distance\":%.3lf,\"angle\":%.3lf},"
        "\"speed_order\":{\"distance\":%.3lf,\"angle\":%.3lf}",
        ctrl->control.current_mode,
        ctrl->control.pose_current.O(), ctrl->control.pose_current.x(), ctrl->control.pose_current.y(),
        ctrl->control.pose_order.O(), ctrl->control.pose_order.x(), ctrl->control.pose_order.y(),
        ctrl->control.current_cycle,
        ctrl->control.speed_current.distance(), ctrl->control.speed_current.angle(),
        ctrl->control.speed_order.distance(), ctrl->control.speed_order.angle()
        );

    out.printf(",\"path\":");
    avoidance_print_path(out);

    out.printf(",\"obstacles\":");
    cogip::obstacles::print_all_json(out);

    out.printf("}\n");

    out.unlock();
}

void pf_init_quadpid_params(ctrl_quadpid_parameters_t ctrl_quadpid_params)
{
    ctrl_quadpid.quadpid_params = ctrl_quadpid_params;
}

ctrl_quadpid_t *pf_get_quadpid_ctrl(void)
{
    return &ctrl_quadpid;
}

ctrl_t *pf_get_ctrl(void)
{
    return (ctrl_t *)&ctrl_quadpid;
}

void pf_ctrl_pre_running_cb(cogip::cogip_defs::Pose &robot_pose,
                            cogip::cogip_defs::Polar &robot_speed,
                            cogip::cogip_defs::Polar &motor_command)
{
    (void)motor_command;

    /* catch speed */
    encoder_read(robot_speed);

    /* convert to position */
    odometry_update(robot_pose, robot_speed, SEGMENT);
}

void pf_ctrl_post_stop_cb(cogip::cogip_defs::Pose &robot_pose,
                          cogip::cogip_defs::Polar &robot_speed,
                          cogip::cogip_defs::Polar &motor_command)
{
    (void)robot_pose;
    (void)robot_speed;

    /* Set distance and angle command to 0 to stop the robot*/
    motor_command.set_distance(0);
    motor_command.set_angle(0);

    /* Send command to motors */
    motor_drive(motor_command);
}

void pf_ctrl_post_running_cb(cogip::cogip_defs::Pose &robot_pose,
                             cogip::cogip_defs::Polar &robot_speed,
                             cogip::cogip_defs::Polar &motor_command)
{
    (void)robot_pose;
    (void)robot_speed;

    /* Send command to motors */
    motor_drive(motor_command);
}

int encoder_read(cogip::cogip_defs::Polar &robot_speed)
{
    int32_t left_speed = qdec_read_and_reset(HBRIDGE_MOTOR_LEFT) * QDEC_LEFT_POLARITY;
    int32_t right_speed = qdec_read_and_reset(HBRIDGE_MOTOR_RIGHT) * QDEC_RIGHT_POLARITY;

    /* update speed */
    robot_speed.set_distance(((right_speed + left_speed) / 2.0) / PULSE_PER_MM);
    robot_speed.set_angle((right_speed - left_speed) / PULSE_PER_DEGREE);

    return 0;
}

void encoder_reset(void)
{
    qdec_read_and_reset(HBRIDGE_MOTOR_LEFT);
    qdec_read_and_reset(HBRIDGE_MOTOR_RIGHT);
}

void motor_drive(const cogip::cogip_defs::Polar &command)
{
    int16_t right_command = (int16_t) (command.distance() + command.angle());
    int16_t left_command = (int16_t) (command.distance() - command.angle());

    motor_set(MOTOR_DRIVER_DEV(0), HBRIDGE_MOTOR_LEFT, left_command);
    motor_set(MOTOR_DRIVER_DEV(0), HBRIDGE_MOTOR_RIGHT, right_command);
}

int pf_is_camp_left(void)
{
    /* Color switch for coords translations */
    return 1;
}

cogip::obstacles::List *pf_get_dyn_obstacles(void)
{
    return lidar_obstacles;
}

static void *_task_countdown(void *arg)
{
    (void)arg;
    static int countdown = GAME_DURATION_SEC;

    for (;;) {
        xtimer_ticks32_t loop_start_time = xtimer_now();
        if (countdown < 0) {
            planner->stop();
        }
        else {
            DEBUG("GAME TIME: %d\n", countdown);
            countdown--;
        }
        xtimer_periodic_wakeup(&loop_start_time, US_PER_SEC);
    }

    return NULL;
}

static void *_task_planner_start_cancel(void *arg)
{
    bool *start_shell = (bool *)arg;

    /* Wait for Enter to be pressed */
    getchar();

    /* Set a flag and return once done */
    *start_shell = false;

    cogip::tracefd::out.logf("Key pressed, do not start planner.");

    return NULL;
}

void pf_init(void)
{
#ifdef MODULE_SHELL_PLATFORMS
    pf_shell_init();
#endif /* MODULE_SHELL_PLATFORMS */

    /* Debug LED */
    if (gpio_init(GPIO_DEBUG_LED, GPIO_OUT)) {
        puts("WARNING: GPIO_DEBUG_LED not initialized!");
    }
    gpio_clear(GPIO_DEBUG_LED);

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

    gpio_clear(GPIO_DEBUG_LED);

    /*ctrl_set_anti_blocking_on(pf_get_ctrl(), TRUE);*/

    /* Initialize planner */
    planner = new cogip::planners::AstarPlanner(pf_get_ctrl(), app_get_path());

#ifdef MODULE_SHELL_QUADPID
    ctrl_quadpid_shell_init(&ctrl_quadpid);
#endif /* MODULE_SHELL_QUADPID */
}

void pf_init_tasks(void)
{
    ctrl_t *controller = pf_get_ctrl();
    bool start_planner = true;
    int countdown = PF_START_COUNTDOWN;

    lidar_start(LIDAR_MAX_DISTANCE, LIDAR_MINIMUN_INTENSITY);

    obstacle_updater_start(ctrl_get_pose_current(controller));

    /* Create thread that up a flag on key pressed to not start the planner automatically */
    kernel_pid_t planner_start_cancel_pid = thread_create(
        planner_start_cancel_thread_stack,
        sizeof(planner_start_cancel_thread_stack),
        THREAD_PRIORITY_MAIN + 1, 0,
        _task_planner_start_cancel,
        &start_planner,
        "wait planner start cancel"
        );

    cogip::tracefd::out.logf("Press Enter to cancel planner start...");

    /* Wait for Enter key pressed or countdown */
    while ((start_planner) && (countdown > 0)) {
        xtimer_ticks32_t loop_start_time = xtimer_now();
        cogip::tracefd::out.logf("%d seconds left...", countdown);
        countdown--;
        xtimer_periodic_wakeup(&loop_start_time, US_PER_SEC);
    }

    /* Stop task_planner_start_cancel thread if still running */
    thread_t *planner_start_cancel_thread = (thread_t *)thread_get(planner_start_cancel_pid);
    if (planner_start_cancel_thread) {
        sched_set_status(planner_start_cancel_thread, STATUS_STOPPED);
    }

    /* Create controller thread */
    thread_create(
        controller_thread_stack,
        sizeof(controller_thread_stack),
        THREAD_PRIORITY_MAIN - 4, 0,
        task_ctrl_update,
        (void *)controller,
        "motion control"
        );

    planner->start_thread();

    trace_start();

    if (start_planner) {
        /* Debug indicator to track the planner started state */
        gpio_set(GPIO_DEBUG_LED);

        /* Create countdown thread */
        thread_create(
            countdown_thread_stack,
            sizeof(countdown_thread_stack),
            THREAD_PRIORITY_MAIN - 3,
            0,
            _task_countdown,
            NULL,
            "countdown"
            );

        /* Start game */
        DEBUG("platform: Start game\n");
        planner->start();
    }
}
