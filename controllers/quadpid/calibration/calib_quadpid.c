#include <stdio.h>

/* Standard includes */
#include <stdlib.h>

/* RIOT includes */
#include "fmt.h"
#include "xtimer.h"

/* Project includes */
#include "platform.h"
#include "calibration/calib_quadpid.h"

/**
 * @brief Impulse function parameters
 *
 * This structures store all parameters related to impulse speed_order function.
 * These parameters defines the expected impulse response amplitude and timings
 * as visible in following figure.
 *
 * @verbatim
 *
 *                       output
 *                          ^
 *                          |
 *                          |
 *                          |
 * impulse_max_value  --->  |         +--------+
 *                          |         |        |
 *                          |         |        |
 *                          |         |        |
 *                          |         |        |
 *                          +---------+--------+--------------------->   time
 *                        0                                            [seconds]
 *                                    ^        ^                ^
 *                                    |        |                |
 *                                             |
 *                           cycle_start_mot   |          cycle_end_func
 *                                             |
 *                                       cycle_end_mot
 *
 * @endverbatim
 */
typedef struct {
    uint32_t cycle_start_mot;    /**< Defines when output should be 'impulse_max_value', in seconds */
    uint32_t cycle_end_mot;      /**< Defines when output return back to 0, in seconds */
    uint32_t cycle_end_func;     /**< Defines when impulse function is considered finished, in seconds */
    double   impulse_max_value;  /**< Maximum output when impulse is 'high' level. ('low' lelvel is 0). */

    uint8_t  is_linear;          /**< TRUE if linear set_point required, FALSE for angular */
} impulse_cfg_t;


/* 20ms period == 50Hz sampling rate */
#define PULSE_PER_SEC              50

/* Speed correction calibration usage */
static void ctrl_quadpid_speed_calib_print_usage(void)
{
    puts(">>> Entering calibration for quadpid controller");

    puts("\t'q'\t Quit calibration");
    puts("\t'r'\t Send reset");
    puts("\t'L'\t Linear speed PID test");
    puts("\t'A'\t Angular speed PID test");
    puts("\t'p'\t Set linear Kp");
    puts("\t'i'\t Set linear Ki");
    puts("\t'd'\t Set linear Kd");
    puts("\t'P'\t Set angular Kp");
    puts("\t'I'\t Set angular Ki");
    puts("\t'D'\t Set angular Kd");
    puts("\t'r'\t Reset PID coefficients to (Kp = 1, Ki = 0, Kd = 0)");
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

static impulse_cfg_t impulse_cfg__speed_pid_linear = {
    .cycle_start_mot = 1 /* sec */,
    .cycle_end_mot   = 3 /* sec */,
    .cycle_end_func  = 5 /* sec */,
    .impulse_max_value = 35,
    .is_linear       = TRUE,
};

static impulse_cfg_t impulse_cfg__speed_pid_angular = {
    .cycle_start_mot = 1 /* sec */,
    .cycle_end_mot   = 3 /* sec */,
    .cycle_end_func  = 5 /* sec */,
    .impulse_max_value = 13,
    .is_linear       = FALSE,
};

/**
 * @brief Impulse function
 *
 * This compute the output of the impulse function given a time value.
 *
 * @param[in]        cfg        Impulse function parameters, @ref impulse_cfg_t.
 * @param[in]       time        Time [cycle number].
 * @param[out] set_point        Impulse function output.
 *
 * @return TRUE if time is superior or equal to cycle_end_func time marker, FALSE otherwise.
 */
static uint8_t func_impulse(impulse_cfg_t *cfg, uint16_t time, double *set_point)
{
    /* Timeline is before impulse start marker, motor set to 0 */
    if (time < cfg->cycle_start_mot * PULSE_PER_SEC)
        *set_point = 0;
    /* Timeline is in impulse interval, motor set to impulse_max_value */
    else if (time >= cfg->cycle_start_mot * PULSE_PER_SEC && time < cfg->cycle_end_mot * PULSE_PER_SEC)
        *set_point = cfg->impulse_max_value;
    /* Timeline is after impulse end marker, motor reset to 0 */
    else if (time >= cfg->cycle_end_mot * PULSE_PER_SEC && time < cfg->cycle_end_func * PULSE_PER_SEC)
        *set_point = 0;

    return (time >= cfg->cycle_end_func * PULSE_PER_SEC);
}

/**
 * @brief Variable speed_order impulse function
 *
 * This generate a variable speed_order over time, using impulse function.
 *
 * @param[in]   user_data       Impulse function parameters, @ref impulse_cfg_t.
 * @param[in]        time       Time [cycle number].
 * @param[out] speed_order      Computed speed_order value.
 *
 * @return TRUE if function generation is complete, FALSE otherwise.
 */
static uint8_t func_impulse_on_speed_order(void *user_data, uint16_t time, polar_t *speed_order)
{
    impulse_cfg_t *cfg = (impulse_cfg_t *)user_data;
    double set_point = 0;

    uint8_t seq_finished = func_impulse(cfg, time, &set_point);

    if (cfg->is_linear) {
        speed_order->distance = set_point;
        speed_order->angle = 0;
    }
    else {
        speed_order->distance = 0;
        speed_order->angle = set_point;
    }

    return seq_finished;
}


/**
 * @brief Speed calibration sequence
 *
 * Start controller with speed feedback loop PID only. This rely on the use
 * of variable speed_order previously registered in the controller and so does
 * not apply any constant speed_order. Test sequence is launched until variable
 * speed_order function is finished, or until a timeout occured.
 *
 * @param[in]   ctrl_quadpid    Controller object.
 */
static void calib_seq_speed_pid_only(ctrl_t* ctrl_quadpid)
{
    /* Turn controller into runnning mode swith only speed correction loops */
    ctrl_set_mode(ctrl_quadpid, CTRL_MODE_RUNNING_SPEED);

    /* Wait for sequence to finish or timeout */
    uint16_t timeout = 20; /* seconds */
    do {
        xtimer_usleep(US_PER_SEC);
    } while (ctrl_get_mode(ctrl_quadpid) != CTRL_MODE_STOP && --timeout);

    if (!timeout) {
        puts("Unexpected behavior");
        ctrl_set_mode(ctrl_quadpid, CTRL_MODE_STOP);
    }
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

/**
 * @brief Reset all PID coefficients before calibration
 *
 * Set all PID to Kp = 1, Ki = 0, Kd = 0.
 *
 * @param[in]   ctrl_quadpid    Controller object.
 */
static void pid_reset_all(ctrl_quadpid_t *ctrl_quadpid)
{
    PID_t pid_initial = {
        .kp = 1.0,
        .ki = 0.0,
        .kd = 0.0,
    };

    /* Reset all PIDs */
    ctrl_quadpid->quadpid_params.linear_speed_pid  = pid_initial;
    ctrl_quadpid->quadpid_params.angular_speed_pid = pid_initial;
    ctrl_quadpid->quadpid_params.linear_pose_pid   = pid_initial;
    ctrl_quadpid->quadpid_params.angular_pose_pid  = pid_initial;
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

    /* Get the quadpid controller */
    ctrl_quadpid_t* ctrl_quadpid = pf_get_quadpid_ctrl();

    /* Key pressed */
    char c = 0;

    while (c != 'q') {
        /* Wait for a key pressed */
        c = getchar();

        encoder_reset();

        switch(c) {
            /* Linear speed PID test. */
            case 'L':
                ctrl_register_speed_order_cb((ctrl_t*)ctrl_quadpid,
                                             func_impulse_on_speed_order,
                                             &impulse_cfg__speed_pid_linear);

                calib_seq_speed_pid_only((ctrl_t*)ctrl_quadpid);

                ctrl_register_speed_order_cb((ctrl_t*)ctrl_quadpid, NULL, NULL);
                break;
            /* Angular speed PID test. */
            case 'A':
                ctrl_register_speed_order_cb((ctrl_t*)ctrl_quadpid,
                                             func_impulse_on_speed_order,
                                             &impulse_cfg__speed_pid_angular);

                calib_seq_speed_pid_only((ctrl_t*)ctrl_quadpid);

                ctrl_register_speed_order_cb((ctrl_t*)ctrl_quadpid, NULL, NULL);
                break;
            /* Linear speed Kp */
            case 'p':
                printf("Enter new linear speed Kp (%0.2lf):\n", ctrl_quadpid->quadpid_params.linear_speed_pid.kp);
                scanf("%lf", &ctrl_quadpid->quadpid_params.linear_speed_pid.kp);
                break;
            /* Linear speed Ki */
            case 'i':
                printf("Enter new linear speed Ki (%0.2lf):\n", ctrl_quadpid->quadpid_params.linear_speed_pid.ki);
                scanf("%lf", &ctrl_quadpid->quadpid_params.linear_speed_pid.ki);
                break;
            /* Linear speed Kd */
            case 'd':
                printf("Enter new linear speed Kd (%0.2lf):\n", ctrl_quadpid->quadpid_params.linear_speed_pid.kd);
                scanf("%lf", &ctrl_quadpid->quadpid_params.linear_speed_pid.kd);
                break;
            /* Angular speed Kp */
            case 'P':
                printf("Enter new angular speed Kp (%0.2lf):\n", ctrl_quadpid->quadpid_params.angular_speed_pid.kp);
                scanf("%lf", &ctrl_quadpid->quadpid_params.angular_speed_pid.kp);
                break;
            /* Angular speed Ki */
            case 'I':
                printf("Enter new angular speed Ki (%0.2lf):\n", ctrl_quadpid->quadpid_params.angular_speed_pid.ki);
                scanf("%lf", &ctrl_quadpid->quadpid_params.angular_speed_pid.ki);
                break;
            /* Angular speed Kd */
            case 'D':
                printf("Enter new angular speed Kd (%0.2lf):\n", ctrl_quadpid->quadpid_params.angular_speed_pid.kd);
                scanf("%lf", &ctrl_quadpid->quadpid_params.angular_speed_pid.kd);
                break;
            case 'r':
                pid_reset_all(ctrl_quadpid);
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
