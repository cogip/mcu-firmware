/* System includes */
#include <stdio.h>
#include <stdlib.h>

/* Project includes */
#include "cogip_defs.h"
#include "shell_menu.h"
#include "obstacles.h"

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
    .default_radius = 0,
    .min_distance = 0,
};

uint32_t cycle = 1;

pose_t robot_state = {
    .x = 0.0,
    .y = 1000.0,
    .O = 0.0
};

static void _init_border_obstacles(void)
{
    border_obstacles = obstacles_init(&border_obstacles_params);

    obstacles_add(border_obstacles, (obstacle_t){
        .type = OBSTACLE_POLYGON,
        .form.polygon = {
            .points = {
                (pose_t){ .x = -1500, .y = -10 },
                (pose_t){ .x = -1500, .y = 0 },
                (pose_t){ .x = 1500, .y = 0 },
                (pose_t){ .x = 1500, .y = -10 }
            },
            .count = 4,
        },
    });

    obstacles_add(border_obstacles, (obstacle_t){
        .type = OBSTACLE_POLYGON,
        .form.polygon = {
            .points = {
                (pose_t){ .x = -1500, .y = 2000 },
                (pose_t){ .x = -1500, .y = 2010 },
                (pose_t){ .x = 1500, .y = 2010 },
                (pose_t){ .x = 1500, .y = 2000 }
            },
            .count = 4,
        },
    });

    obstacles_add(border_obstacles, (obstacle_t){
        .type = OBSTACLE_POLYGON,
        .form.polygon = {
            .points = {
                (pose_t){ .x = -1500, .y = 0 },
                (pose_t){ .x = -1500, .y = 2000 },
                (pose_t){ .x = -1510, .y = 2000 },
                (pose_t){ .x = -1510, .y = 0 }
            },
            .count = 4,
        },
    });

    obstacles_add(border_obstacles, (obstacle_t){
        .type = OBSTACLE_POLYGON,
        .form.polygon = {
            .points = {
                (pose_t){ .x = 1500, .y = 0 },
                (pose_t){ .x = 1500, .y = 2000 },
                (pose_t){ .x = 1510, .y = 2000 },
                (pose_t){ .x = 1510, .y = 0 }
            },
            .count = 4,
        },
    });
}

/* Shell commands */
static int _cmd_print_state(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    printf(
        "{"
        "\"mode\": 0, "
        "\"pose_current\": {\"x\": %lf, \"y\": %lf, \"O\": %lf}, "
        "\"pose_order\": {\"x\": %lf, \"y\": %lf, \"O\": %lf}, "
        "\"cycle\": %" PRIu32
        "}\n",
        robot_state.x, robot_state.y, robot_state.O,
        robot_state.x, robot_state.y, robot_state.O,
        cycle
        );

    return EXIT_SUCCESS;
}

int _cmd_print_obstacles(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    obstacles_print_all_json(stdout);

    return EXIT_SUCCESS;
}

int main(void)
{
    puts("\n== Lidar obstacle detection example ==");

    /* This print is required by the simulator */
    puts("Press Enter to cancel planner start...");

    /* Add print data command */
    const shell_command_t shell_commands[] = {
        { "_state", "Print current state", _cmd_print_state },
        { "_dyn_obstacles", "Print obstacles", _cmd_print_obstacles },
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

    /* Start shell */
    menu_start();

    return 0;
}
