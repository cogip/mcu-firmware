/* System includes */
#include <thread.h>
#include <stdlib.h>
#include <string.h>

/* RIOT includes */
#define ENABLE_DEBUG        (0)
#include "debug.h"
#include "log.h"
#include "shell.h"
#include "xtimer.h"

/* Project includes */
#include "calibration/calib_pca9548.h"
#include "calibration/calib_planner.h"
#include "calibration/calib_platform.h"
#include "calibration/calib_quadpid.h"
#include "avoidance.h"
#include "ctrl.h"

/* Controller */
static ctrl_t* ctrl = NULL;

/* Thread stacks */
char start_shell_thread_stack[THREAD_STACKSIZE_LARGE];

/* Shell command array */
static shell_command_linked_t current_shell_commands;

shell_command_linked_t pf_shell_commands;

void pf_push_shell_commands(shell_command_linked_t *shell_commands) {
    shell_commands->previous = current_shell_commands.current;
    memcpy(&current_shell_commands, shell_commands, sizeof(shell_command_linked_t));
    printf("Enter shell menu: %s\n", current_shell_commands.name);
}

void pf_pop_shell_commands(void) {
    printf("Exit shell menu: %s\n", current_shell_commands.name);
    if(current_shell_commands.previous) {
        memcpy(&current_shell_commands, current_shell_commands.previous, sizeof(shell_command_linked_t));
    }
    printf("Enter shell menu: %s\n", current_shell_commands.name);
}

void pf_init_shell_commands(shell_command_linked_t *shell_commands, const char *name) {
    shell_commands->name = name;
    shell_commands->current = shell_commands;
    shell_commands->previous = NULL;

    for(uint8_t i = 0 ; i < NB_SHELL_COMMANDS ; i++) {
        shell_commands->shell_commands[i] = (shell_command_t){ NULL, NULL, NULL };
    }
    shell_commands->shell_commands[0] = cmd_help_json;
    shell_commands->shell_commands[1] = cmd_print_state;
    shell_commands->shell_commands[2] = cmd_print_dyn_obstacles;
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

    printf("{\"name\": \"%s\", \"entries\": [", current_shell_commands.name);
    shell_command_t * entry = (shell_command_t*)&current_shell_commands;
    for (; entry->name != NULL; entry++) {
        if(entry != (shell_command_t*)&current_shell_commands) {
            printf(", ");
        }
        printf(
            "{\"cmd\": \"%s\", \"desc\": \"%s\"}",
            entry->name,
            entry->desc
        );
    }
    printf("]}\n");
    return EXIT_SUCCESS;
}

int pf_exit_shell(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    pf_pop_shell_commands();
    return EXIT_SUCCESS;
}

int pf_print_state(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    printf(
        "{"
          "\"mode\": \"%u\", "
          "\"pose_current\": "
          "{"
            "\"O\": \"%lf\", "
            "\"x\": \"%lf\", "
            "\"y\": \"%lf\""
          "}, "
          "\"pose_order\": "
          "{"
            "\"O\": \"%lf\", "
            "\"x\": \"%lf\", "
            "\"y\": \"%lf\""
          "}, "
          "\"cycle\": \"%"PRIu32"\", "
          "\"speed_current\": "
          "{"
            "\"distance\": \"%lf\", "
            "\"angle\": \"%lf\""
          "}, "
          "\"speed_order\": "
          "{"
            "\"distance\": \"%lf\", "
            "\"angle\": \"%lf\""
          "}"
        "}\n",
        ctrl_quadpid.control.current_mode,
        ctrl_quadpid.control.pose_current.O,
        ctrl_quadpid.control.pose_current.x,
        ctrl_quadpid.control.pose_current.y,
        ctrl_quadpid.control.pose_order.O,
        ctrl_quadpid.control.pose_order.x,
        ctrl_quadpid.control.pose_order.y,
        ctrl_quadpid.control.current_cycle,
        ctrl_quadpid.control.speed_current.distance,
        ctrl_quadpid.control.speed_current.angle,
        ctrl_quadpid.control.speed_order.distance,
        ctrl_quadpid.control.speed_order.angle
    );

    return EXIT_SUCCESS;
}

shell_command_t cmd_exit_shell = {
    "exit", "Exit planner calibration",
    pf_exit_shell
};

shell_command_t cmd_help_json = {
    "_help_json", "Display available commands in JSON format",
    pf_display_json_help
};

shell_command_t cmd_print_state = {
    "_state", "Print current state",
    pf_print_state
};

shell_command_t cmd_print_dyn_obstacles = {
    "_dyn_obstacles", "Print dynamic obstacles",
    avoidance_print_dyn_obstacles
};

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

void pf_init_calib_tasks(ctrl_t* pf_ctrl)
{
    static int start_shell = FALSE;

    int countdown = PF_START_COUNTDOWN;

    ctrl = pf_ctrl;

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

    /* If Enter was pressed, start shell */
    if (start_shell) {
        /* Define buffer to be used by the shell */
        char line_buf[SHELL_DEFAULT_BUFSIZE];

        pf_push_shell_commands(&pf_shell_commands);

        /* Start shell */
        shell_run((shell_command_t*)&current_shell_commands, line_buf, SHELL_DEFAULT_BUFSIZE);
    }
    else {
        /* Stop useless task_start_shell thread still running */
        thread_t* start_shell_thread = (thread_t*)thread_get(start_shell_pid);
        if (start_shell_thread) {
            sched_set_status(start_shell_thread, STATUS_STOPPED);
        }
    }
}

void pf_calib_init(void)
{
    pf_init_shell_commands(&pf_shell_commands, MCUFIRMWARE_PLATFORM_BASE);

    ctrl_quadpid_calib_init();
    pca9548_calib_init();
    pln_calib_init();
}
