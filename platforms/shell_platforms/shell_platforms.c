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
#include "avoidance.h"
#include "ctrl.h"
#include "shell_platforms.h"


#define GLOBAL_MENU "_global"

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
    if ((shell_commands) && (shell_commands->name))
        return;

    shell_commands->name = name;
    shell_commands->current = shell_commands;
    shell_commands->previous = NULL;

    for(uint8_t i = 0 ; i < NB_SHELL_COMMANDS ; i++) {
        shell_commands->shell_commands[i] = (shell_command_t){ NULL, NULL, NULL };
    }

    pf_add_shell_command(shell_commands, &cmd_help_json);
    pf_add_shell_command(shell_commands, &cmd_print_state);
    pf_add_shell_command(shell_commands, &cmd_print_dyn_obstacles);
    pf_add_shell_command(shell_commands, &cmd_motors_test);
}

void pf_add_shell_command(shell_command_linked_t *shell_commands, const shell_command_t *command)
{
    uint8_t command_id = 0;

    if ((!shell_commands) || (!shell_commands->name)) {
        pf_init_shell_commands(shell_commands, GLOBAL_MENU);
    }

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
        ctrl->control.current_mode,
        ctrl->control.pose_current.O,
        ctrl->control.pose_current.x,
        ctrl->control.pose_current.y,
        ctrl->control.pose_order.O,
        ctrl->control.pose_order.x,
        ctrl->control.pose_order.y,
        ctrl->control.current_cycle,
        ctrl->control.speed_current.distance,
        ctrl->control.speed_current.angle,
        ctrl->control.speed_order.distance,
        ctrl->control.speed_order.angle
    );

    return EXIT_SUCCESS;
}

int pf_motors_test(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    int nb_motors = 0;

    ctrl_set_mode(ctrl, CTRL_MODE_IDLE);

    for (motor_driver_t i = 0; i < MOTOR_DRIVER_NUMOF; i++) {
        int nb_motors_tmp = motor_driver_config[i].nb_motors;
        int pwm_resolution = motor_driver_config[i].pwm_resolution;

        for (int j = 0; j < nb_motors_tmp; j++) {

            int32_t qdec_value = 0;
            int timeout = 3000;

            printf("### Testing motor %d of motor driver %d\n", j, i);

            /* Reset qdec */
            qdec_read_and_reset(QDEC_DEV(nb_motors + j));

            /* Forward */
            puts("    Forward move");
            motor_set(i, j, pwm_resolution / 4);
            while (timeout--) {
                qdec_value += qdec_read_and_reset(QDEC_DEV(nb_motors + j));
                xtimer_usleep(US_PER_MS);
            }
            printf("    qdec value = %ld\n", qdec_value);
            puts("    Done");

            /* Stop */
            timeout = 3000;
            qdec_value = 0;
            puts("    Stop");
            motor_set(i, j, 0);
            xtimer_usleep(3 * US_PER_SEC);
            puts("    Done");

            /* Backward */
            puts("    Backward move");
            motor_set(i, j, -pwm_resolution / 4);
            while (timeout--) {
                qdec_value += qdec_read_and_reset(QDEC_DEV(nb_motors + j));
                xtimer_usleep(US_PER_MS);
            }
            printf("    qdec value = %ld\n", qdec_value);
            puts("    Done");

            /* Stop */
            puts("    Stop");
            motor_set(i, j, 0);
            puts("    Done");
        }

        nb_motors += nb_motors_tmp;
    }

    ctrl_set_mode(ctrl, CTRL_MODE_STOP);

    return EXIT_SUCCESS;
}

shell_command_t cmd_exit_shell = {
    "exit", "Exit planner shell",
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

shell_command_t cmd_motors_test = {
    "mt", "Test all DC motors",
    pf_motors_test
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

    puts("Entering shell mode...");

    return NULL;
}

void pf_init_shell_tasks(ctrl_t* pf_ctrl)
{
    static int start_shell = FALSE;

    int countdown = PF_START_COUNTDOWN;

    ctrl = pf_ctrl;

    puts("Built-in shell mode is ACTIVATED");

    /* Create thread that up a flag on key pressed to start a shell instead of
       planner below */
    kernel_pid_t start_shell_pid = thread_create(start_shell_thread_stack,
                  sizeof(start_shell_thread_stack),
                  THREAD_PRIORITY_MAIN + 1, 0,
                  pf_task_start_shell, &start_shell, "shell");

    puts("Press Enter to enter shell mode...");

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

void pf_shell_init(void)
{
    pf_init_shell_commands(&pf_shell_commands, GLOBAL_MENU);
}

