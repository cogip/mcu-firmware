/* Firmware includes */
#include "shell.h"
#include "shell_commands.h"
#include "vacuum_pump.h"
#include "vacuum_pump_params.h"

/* System includes */
#include <stdio.h>
#include <stdlib.h>

#define SHELL_BUFSIZE (128U)

static int parse_dev(char *arg)
{
    int dev = atoi(arg);

    if (dev < 0 || (int)ARRAY_SIZE(vacuum_pump_params) < dev) {
        printf("Error: Invalid device id (%s)\n", arg);
        return -1;
    }
    return dev;
}

static int cmd_init(int argc, char **argv)
{
    vacuum_pump_t vacuum_pump;
    int id = -1;

    if (argc != 2) {
        printf("usage: %s <dev_id>\n", argv[0]);
        return 1;
    }
    /* parse parameters */
    id = parse_dev(argv[1]);
    if (id < 0) {
        return -1;
    }

    vacuum_pump = id;

    vacuum_pump_init(vacuum_pump, &vacuum_pump_params[vacuum_pump]);

    return 0;
}

static int cmd_start(int argc, char **argv)
{
    vacuum_pump_t vacuum_pump;
    int id = -1;

    if (argc != 2) {
        printf("usage: %s <dev_id>\n", argv[0]);
        return 1;
    }
    /* parse parameters */
    id = parse_dev(argv[1]);
    if (id < 0) {
        return -1;
    }

    vacuum_pump = id;

    vacuum_pump_start(vacuum_pump);

    return 0;
}

static int cmd_stop(int argc, char **argv)
{
    vacuum_pump_t vacuum_pump;
    int id = -1;

    if (argc != 2) {
        printf("usage: %s <dev_id>\n", argv[0]);
        return 1;
    }
    /* parse parameters */
    id = parse_dev(argv[1]);
    if (id < 0) {
        return -1;
    }

    vacuum_pump = id;

    vacuum_pump_stop(vacuum_pump);

    return 0;
}

static int cmd_test(int argc, char **argv)
{
    vacuum_pump_t vacuum_pump;
    int id = -1;

    if (argc != 2) {
        printf("usage: %s <dev_id>\n", argv[0]);
        return 1;
    }
    /* parse parameters */
    id = parse_dev(argv[1]);
    if (id < 0) {
        return -1;
    }

    vacuum_pump = id;

    printf("Test pump %u = %d\n", vacuum_pump, vacuum_pump_is_under_pressure(vacuum_pump));

    return 0;
}

/* Shell commands and callbacks */
static const shell_command_t shell_commands[] = {
    { "init", "Init vacuum_pump device", cmd_init },
    { "start", "Start vacuum_pump device", cmd_start },
    { "stop", "Stop vacuum_pump device", cmd_stop },
    { "test", "Print current values", cmd_test },
    { NULL, NULL, NULL }
};

int main(void)
{
    puts("\n== vacuum_pump example ==");

    /* Run the shell */
    char line_buf[SHELL_BUFSIZE];

    shell_run(shell_commands, line_buf, SHELL_BUFSIZE);
    return 0;
}
