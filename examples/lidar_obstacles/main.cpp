/* System includes */
#include <stdio.h>
#include <stdlib.h>

/* RIOT includes */
#include "xtimer.h"

/* Project includes */
#include "cogip_defs.h"
#include "shell_menu.h"
#include "obstacles.h"
#include "tracefd.h"

/* Application includes */
#include "common_defs.h"
#include "lidar_utils.h"
#include "lidar_obstacles.h"

#ifdef MODULE_SHMEM
#include "shmem.h"
#endif

#define LIDAR_MINIMUN_INTENSITY 1000

/* Obstacles id of border obstacles*/
obstacles_t border_obstacles = OBSTACLES_NUMOF;

/* Border obstacles parameters */
obstacles_params_t border_obstacles_params = {
    .default_circle_radius = 0,
    .min_distance = 0,
};

uint32_t cycle = 1;

pose_t robot_state = {
    .coords.x = 0.0,
    .coords.y = 1000.0,
    .O = 0.0
};

static bool trace_on = false;

/* Periodic task */
#define TASK_PERIOD_MS 100

/* Thread stack */
static char trace_thread_stack[THREAD_STACKSIZE_MEDIUM];

/* Thread priority */
#define TRACE_PRIO (THREAD_PRIORITY_MAIN - 1)

static void _init_border_obstacles(void)
{
    border_obstacles = obstacles_init(&border_obstacles_params);

    obstacles_add(border_obstacles, obstacles_rectangle_init(
                      (coords_t) { .x = 0, .y = -5 }, 3000, 10, 0));

    obstacles_add(border_obstacles, obstacles_rectangle_init(
                      (coords_t) { .x = 0, .y = 2005 }, 3000, 10, 0));

    obstacles_add(border_obstacles, obstacles_rectangle_init(
                      (coords_t) { .x = -1505, .y = 1000 }, 10, 2000, 0));

    obstacles_add(border_obstacles, obstacles_rectangle_init(
                      (coords_t) { .x = 1505, .y = 1000 }, 10, 2000, 0));

}

static int _cmd_trace_on_off(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    if (trace_on) {
        puts("Deactivate traces");
        menu_rename_command("_trace_off", "_trace_on");
        trace_on = false;
    }
    else {
        puts("Activate traces");
        menu_rename_command("_trace_on", "_trace_off");
        trace_on = true;
    }
    return 0;
}

static void _print_state(void)
{
    tracefd_lock(tracefd_stdout);

    tracefd_printf(
        tracefd_stdout,
        "{"
        "\"mode\":0,"
        "\"pose_current\":{\"x\":%.3lf,\"y\":%.3lf,\"O\":%.3lf},"
        "\"pose_order\":{\"x\":%.3lf,\"y\":%.3lf,\"O\":%.3lf},"
        "\"cycle\":%" PRIu32,
        robot_state.coords.x, robot_state.coords.y, robot_state.O,
        robot_state.coords.x, robot_state.coords.y, robot_state.O,
        cycle
        );

    tracefd_printf(tracefd_stdout, ",\"obstacles\":");
    obstacles_print_all_json(tracefd_stdout);

    tracefd_printf(tracefd_stdout, "}\n");

    tracefd_unlock(tracefd_stdout);
}

/* Thread loop */
static void *_thread_trace(void *arg)
{
    (void)arg;

    for (;;) {
        xtimer_ticks32_t loop_start_time = xtimer_now();

        if (trace_on) {
            _print_state();
        }

        cycle++;
        xtimer_periodic_wakeup(&loop_start_time, TASK_PERIOD_MS * US_PER_MS);
    }

    return NULL;
}

int main(void)
{
    tracefd_jlog(tracefd_stdout, "== Lidar obstacle detection example ==");

    /* Add print data command */
    const shell_command_t shell_commands[] = {
        { "_trace_on", "Activate/deactivate trace", _cmd_trace_on_off },
        PRINT_LIDAR_DATA_CMD,
#ifdef MODULE_SHMEM
        SHMEM_SET_KEY_CMD,
#endif
        MENU_NULL_CMD
    };
    menu_add_list(menu_root, shell_commands);

    _init_border_obstacles();

    lidar_start(LIDAR_MAX_DISTANCE, LIDAR_MINIMUN_INTENSITY);

    obstacle_updater_start(&robot_state);

    /* Start the trace thread */
    thread_create(
        trace_thread_stack,
        sizeof(trace_thread_stack),
        TRACE_PRIO,
        0,
        _thread_trace,
        NULL,
        "Trace thread"
        );

    /* Start shell */
    menu_start();

    return 0;
}
