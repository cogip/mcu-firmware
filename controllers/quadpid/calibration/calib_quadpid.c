#include <stdio.h>

/* Standard includes */
#include <stdlib.h>

/* RIOT includes */
#include "fmt.h"
#include "xtimer.h"

/* Project includes */
#include "platform.h"
#include "calibration/calib_quadpid.h"

/* Speed correction calibration usage */
static void ctrl_quadpid_speed_calib_print_usage(void)
{
    puts(">>> Entering calibration for quadpid controller");

    puts("\t'q'\t Quit calibration");
    puts("\t'r'\t Send reset");
    puts("\t'p'\t Speed linear Kp calibration");
    puts("\t'i'\t Speed linear Ki calibration");
    puts("\t'd'\t Speed linear Kd calibration");
    puts("\t'P'\t Speed angular Kp calibration");
    puts("\t'I'\t Speed angular Ki calibration");
    puts("\t'D'\t Speed angular Kd calibration");
}

/* Position correction calibration usage */
static void ctrl_quadpid_pose_calib_print_usage(void)
{
    puts(">>> Entering calibration for quadpid controller");

    puts("\t'q'\t Quit calibration");
    puts("\t'r'\t Send reset");
    puts("\t'a'\t Speed linear Kp calibration");
    puts("\t'A'\t Speed angular Kp calibration");
}

/* Speed calibration sequence */
static void ctrl_quadpid_speed_calib_seq(ctrl_t* ctrl_quadpid, polar_t* speed_order)
{
    /* Automatic reverse */
    static int dir = -1;
    dir *= -1;

    /* Revert speed order to avoid always going in the same direction */
    speed_order->distance *= dir;
    speed_order->angle *= dir;

    /* Turn controller into runnning mode swith only speed correction loops */
    ctrl_set_mode(ctrl_quadpid, CTRL_MODE_RUNNING_SPEED);

    /* Send speed order to the controller */
    ctrl_set_speed_order(ctrl_quadpid, speed_order);

    /* Wait seconds before stopping the controller */
    xtimer_sleep(1);
    ctrl_set_mode(ctrl_quadpid, CTRL_MODE_STOP);
}

/* Position calibration sequence */
static void ctrl_quadpid_pose_calib_seq(ctrl_t* ctrl_quadpid, pose_t* pos)
{
    /* Speed order is fixed to maximum speed */
    polar_t speed_order = {
        .distance = MAX_SPEED,
        .angle = MAX_SPEED / 2,
    };

    /* Send the speed order and the position to reach to the controller */
    ctrl_set_pose_to_reach(ctrl_quadpid, pos);
    ctrl_set_speed_order(ctrl_quadpid, &speed_order);

    /* Turn the controller into the running mode */
    ctrl_set_mode(ctrl_quadpid, CTRL_MODE_RUNNING);

    /* Wait for position reached */
    while(!ctrl_is_pose_reached(ctrl_quadpid))
        xtimer_usleep(10*US_PER_MS);

    /* Stop the controller */
    ctrl_set_mode(ctrl_quadpid, CTRL_MODE_STOP);
}

/* Speed calibration command */
static int ctrl_quadpid_speed_calib_cmd(int argc, char **argv)
{
    (void)argv;
    int ret = 0;

    /* Always print usage first */
    ctrl_quadpid_speed_calib_print_usage();

    /* Check arguments */
    if (argc > 1) {
        puts("Bad arguments number !");
        ret = -1;
        goto ctrl_quadpid_calib_servo_cmd_err;
    }

    /* Useful to reset all PIDs */
    PID_t pid_zero = {
        .kp=0,
        .ki=0,
        .kd=0,
    };

    /* Get the quadpid controller */
    ctrl_quadpid_t* ctrl_quadpid = pf_get_quadpid_ctrl();

    /* Reset all PIDs, even position ones as they are uneeded */
    ctrl_quadpid->quadpid_params.linear_speed_pid = pid_zero;
    ctrl_quadpid->quadpid_params.angular_speed_pid = pid_zero;
    ctrl_quadpid->quadpid_params.linear_pose_pid = pid_zero;
    ctrl_quadpid->quadpid_params.angular_pose_pid = pid_zero;

    /* Declare a speed order, will be initialized specically for each case */
    polar_t speed_order;

    /* Key pressed */
    char c = 0;

    while (c != 'q') {
        /* Wait for a key pressed */
        c = getchar();

        encoder_reset();

        switch(c) {
            /* Linear speed Kp */
            case 'p':
                speed_order.distance = MAX_SPEED;
                speed_order.angle = 0;
                printf("Enter new angular speed Kp (%0.2lf):\n", ctrl_quadpid->quadpid_params.linear_speed_pid.kp);
                scanf("%lf", &ctrl_quadpid->quadpid_params.linear_speed_pid.kp);
                ctrl_quadpid_speed_calib_seq((ctrl_t*)ctrl_quadpid, &speed_order);
                break;
            /* Linear speed Ki */
            case 'i':
                speed_order.distance = MAX_SPEED;
                speed_order.angle = 0;
                puts("**WARNING**: Setup your optimal Kp before setting Ki");
                printf("Enter new angular speed Kp (%0.2lf):\n", ctrl_quadpid->quadpid_params.linear_speed_pid.ki);
                scanf("%lf", &ctrl_quadpid->quadpid_params.linear_speed_pid.ki);
                ctrl_quadpid_speed_calib_seq((ctrl_t*)ctrl_quadpid, &speed_order);
                break;
            /* Linear speed Kd */
            case 'd':
                speed_order.distance = MAX_SPEED;
                speed_order.angle = 0;
                puts("**WARNING**: Setup your optimal Kp before setting Kd");
                printf("Enter new angular speed Kp (%0.2lf):\n", ctrl_quadpid->quadpid_params.linear_speed_pid.kd);
                scanf("%lf", &ctrl_quadpid->quadpid_params.linear_speed_pid.kd);
                ctrl_quadpid_speed_calib_seq((ctrl_t*)ctrl_quadpid, &speed_order);
                break;
            /* Angular speed Kp */
            case 'P':
                speed_order.distance = 0;
                speed_order.angle = MAX_SPEED / 2;
                printf("Enter new angular speed Kp (%0.2lf):\n", ctrl_quadpid->quadpid_params.angular_speed_pid.kp);
                scanf("%lf", &ctrl_quadpid->quadpid_params.angular_speed_pid.kp);
                ctrl_quadpid_speed_calib_seq((ctrl_t*)ctrl_quadpid, &speed_order);
                break;
            /* Angular speed Ki */
            case 'I':
                speed_order.distance = 0;
                speed_order.angle = MAX_SPEED / 2;
                puts("**WARNING**: Setup your optimal Kp before setting Ki and Kd");
                printf("Enter new angular speed Kp (%0.2lf):\n", ctrl_quadpid->quadpid_params.angular_speed_pid.ki);
                scanf("%lf", &ctrl_quadpid->quadpid_params.angular_speed_pid.ki);
                ctrl_quadpid_speed_calib_seq((ctrl_t*)ctrl_quadpid, &speed_order);
                break;
            /* Angular speed Kd */
            case 'D':
                speed_order.distance = 0;
                speed_order.angle = MAX_SPEED / 2;
                puts("**WARNING**: Setup your optimal Kp before setting Kd");
                printf("Enter new angular speed Kp (%0.2lf):\n", ctrl_quadpid->quadpid_params.angular_speed_pid.kd);
                scanf("%lf", &ctrl_quadpid->quadpid_params.angular_speed_pid.kd);
                ctrl_quadpid_speed_calib_seq((ctrl_t*)ctrl_quadpid, &speed_order);
                break;
            case 'r':
                /* Reset signal, useful for remote application */
                puts("<<<< RESET >>>>");
                break;
            default:
                continue;
        }

        /* Data stop signal */
        puts("<<<< STOP >>>>");

        /* Always remind usage */
        ctrl_quadpid_speed_calib_print_usage();
    }

ctrl_quadpid_calib_servo_cmd_err:
    return ret;
}

/* Position calibration command */
static int ctrl_quadpid_pose_calib_cmd(int argc, char **argv)
{
    (void)argv;
    int ret = 0;

    /* Always print usage first */
    ctrl_quadpid_pose_calib_print_usage();

    /* Check arguments */
    if (argc > 1) {
        puts("Bad arguments number !");
        ret = -1;
        goto ctrl_quadpid_calib_servo_cmd_err;
    }

    /* Calibration path for linear PID */
    static path_pose_t poses_calibration[] = {
        {
            .pos = {
                       .x = 0,
                       .y = 0,
                       .O = 90,
                   },
        },
        {
            .pos = {
                       .x = 500,
                       .y = 500,
                       .O = 0,
                   },
        },
    };

    /* Get the quadpid controller */
    ctrl_quadpid_t* ctrl_quadpid = pf_get_quadpid_ctrl();

    /* Key pressed */
    char c = 0;

    /* Index on position to reach according to current angular one */

    /* Index on position to reach according to current linear one */
    uint8_t pose_linear_index = 0;

    /* Automatic reverse */
    ctrl_set_allow_reverse((ctrl_t*)ctrl_quadpid, TRUE);

    while (c != 'q') {
        /* Wait for a key pressed */
        c = getchar();

        encoder_reset();

        switch(c) {
            /* Linear speed Kp */
                case 'a':
                ctrl_set_pose_current((ctrl_t*)ctrl_quadpid, &poses_calibration[pose_linear_index].pos);
                pose_linear_index ^= 1;
                printf("Enter new linear pose Kp (%0.2lf):\n", ctrl_quadpid->quadpid_params.linear_pose_pid.kp);
                scanf("%lf", &ctrl_quadpid->quadpid_params.linear_pose_pid.kp);
                ctrl_quadpid_pose_calib_seq((ctrl_t*)ctrl_quadpid, &poses_calibration[pose_linear_index].pos);
                break;
            /* Angular pose Kp */
            case 'A':
                ctrl_set_pose_current((ctrl_t*)ctrl_quadpid, &poses_calibration[pose_linear_index].pos);
                pose_linear_index ^= 1;
                printf("Enter new angular pose Kp (%0.2lf):\n", ctrl_quadpid->quadpid_params.angular_pose_pid.kp);
                scanf("%lf", &ctrl_quadpid->quadpid_params.angular_pose_pid.kp);
                ctrl_quadpid_pose_calib_seq((ctrl_t*)ctrl_quadpid, &poses_calibration[pose_linear_index].pos);
                break;
            case 'r':
                /* Reset signal, useful for remote application */
                puts("<<<< RESET >>>>");
                break;
            default:
                continue;
        }

        /* Data stop tag */
        puts("<<<< STOP >>>>");

        /* Always remind usage */
        ctrl_quadpid_pose_calib_print_usage();
    }

ctrl_quadpid_calib_servo_cmd_err:
    return ret;
}

/* Init calibration commands */
void ctrl_quadpid_calib_init(void)
{
    /* Add speed calibration command */
    shell_command_t cmd_calib_speed = {
        "cs", "Speed PID coefficients tuning",
        ctrl_quadpid_speed_calib_cmd
    };

    pf_add_shell_command(&cmd_calib_speed);

    /* Add pose calibration command */
    shell_command_t cmd_calib_pose = {
        "cp", "Pose PID coefficients tuning",
        ctrl_quadpid_pose_calib_cmd
    };

    pf_add_shell_command(&cmd_calib_pose);
}
