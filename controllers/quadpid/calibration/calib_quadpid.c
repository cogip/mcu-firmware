#include <stdio.h>

/* Standard includes */
#include <stdlib.h>

/* RIOT includes */
#include "fmt.h"
#include "thread.h"
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

/* Shell command array */
static shell_command_linked_t ctrl_quadpid_speed_shell_commands;
static const char *quadpid_speed_name = "quadpid_speed";

static shell_command_linked_t ctrl_quadpid_pose_shell_commands;
static const char *quadpid_pose_name = "quadpid_pose";

/* Quadpid controller */
static ctrl_quadpid_t* ctrl_quadpid = NULL;

/* Index on position to reach according to current linear one */
uint8_t pose_linear_index = 0;

static impulse_cfg_t impulse_cfg__tf_ident_linear = {
    .cycle_start_mot = 1 /* sec */,
    .cycle_end_mot   = 3 /* sec */,
    .cycle_end_func  = 5 /* sec */,
    .impulse_max_value = 1000,
    .is_linear       = TRUE,
};

static impulse_cfg_t impulse_cfg__tf_ident_angular = {
    .cycle_start_mot = 1 /* sec */,
    .cycle_end_mot   = 3 /* sec */,
    .cycle_end_func  = 5 /* sec */,
    .impulse_max_value = 1000,
    .is_linear       = FALSE,
};

static impulse_cfg_t impulse_cfg__speed_pid_linear = {
    .cycle_start_mot = 1 /* sec */,
    .cycle_end_mot   = 3 /* sec */,
    .cycle_end_func  = 5 /* sec */,
    .impulse_max_value = MAX_SPEED,
    .is_linear       = TRUE,
};

static impulse_cfg_t impulse_cfg__speed_pid_angular = {
    .cycle_start_mot = 1 /* sec */,
    .cycle_end_mot   = 3 /* sec */,
    .cycle_end_func  = 5 /* sec */,
    .impulse_max_value = MAX_SPEED / 2,
    .is_linear       = FALSE,
};

static impulse_cfg_t* current_impulse_cfg = NULL;
static int seq_finished = FALSE;

/* Thread vars */
static char quadpid_thread_stack[THREAD_STACKSIZE_DEFAULT];
static kernel_pid_t ctrl_quadpid_thread_command_pid = 0;

/**
 * @brief Impulse function
 *
 * This compute the output of the impulse function given a time value.
 *
 * @param[in]       cfg         impulse function parameters, @ref impulse_cfg_t.
 * @param[in]       time        time [cycle number].
 *
 * @return                      computed set point
 */
static double func_impulse(impulse_cfg_t* cfg, uint32_t time)
{
    double set_point = 0;

    if (cfg) {
        /* Timeline is before impulse start marker, motor set to 0 */
        if (time < cfg->cycle_start_mot * PULSE_PER_SEC)
            set_point = 0;
        /* Timeline is in impulse interval, motor set to impulse_max_value */
        else if (time >= cfg->cycle_start_mot * PULSE_PER_SEC && time < cfg->cycle_end_mot * PULSE_PER_SEC)
            set_point = cfg->impulse_max_value;
        /* Timeline is after impulse end marker, motor reset to 0 */
        else if (time >= cfg->cycle_end_mot * PULSE_PER_SEC
                && time < cfg->cycle_end_func * PULSE_PER_SEC)
            set_point = 0;
        else
            seq_finished = TRUE;
    }

    return set_point;
}

/**
 * @brief Variable speed_order impulse function
 *
 * This generate a variable speed_order over time, using impulse function.
 *
 * @param[in]   ctrl        controller object
 *
 * @return                  computed speed order
 */
static polar_t func_impulse_on_speed_order(ctrl_t* ctrl)
{
    double set_point = 0;
    polar_t speed_order = {0, 0};

    set_point = func_impulse(current_impulse_cfg,
            ctrl_get_current_cycle(ctrl));

    if (current_impulse_cfg->is_linear) {
        speed_order.distance = set_point;
        speed_order.angle = 0;
    }
    else {
        speed_order.distance = 0;
        speed_order.angle = set_point;
    }

    return speed_order;
}

/**
 * @brief Transfer function identification sequence
 *
 * Inject impulse command in motor and catch encoders response when no PID are
 * running. This will output data that can be used to generate a mathematical
 * model of the mobile robot.
 *
 * @param[in]   ctrl_quadpid    controller object.
 */
static void calib_seq_identify_robot_tf(ctrl_t* ctrl_quadpid)
{
    seq_finished = FALSE;

    /* Turn controller into direct driving mode, thus no PID at all */
    ctrl_set_mode(ctrl_quadpid, CTRL_MODE_PASSTHROUGH);

    /* Wait for sequence to finish or timeout */
    uint16_t timeout = 20; /* seconds */
    do {
        if (seq_finished == TRUE) {
            ctrl_set_mode(ctrl_quadpid, CTRL_MODE_STOP);
            break;
        }
        xtimer_usleep(US_PER_SEC);
    } while (ctrl_get_mode(ctrl_quadpid) != CTRL_MODE_STOP && --timeout);

    if (!timeout) {
        puts("Unexpected behavior");
        ctrl_set_mode(ctrl_quadpid, CTRL_MODE_STOP);
    }
}

/**
 * @brief Speed calibration sequence
 *
 * Start controller with speed feedback loop PID only. This rely on the use
 * of variable speed_order previously registered in the controller and so does
 * not apply any constant speed_order. Test sequence is launched until variable
 * speed_order function is finished, or until a timeout occured.
 *
 * @param[in]   ctrl_quadpid    controller object.
 */
static void calib_seq_speed_pid_only(ctrl_t* ctrl_quadpid)
{
    seq_finished = FALSE;

    /* Turn controller into runnning mode swith only speed correction loops */
    ctrl_set_mode(ctrl_quadpid, CTRL_MODE_RUNNING_SPEED);

    /* Wait for sequence to finish or timeout */
    uint16_t timeout = 20; /* seconds */
    do {
        if (seq_finished == TRUE) {
            ctrl_set_mode(ctrl_quadpid, CTRL_MODE_STOP);
            break;
        }
        xtimer_usleep(US_PER_SEC);
    } while (ctrl_get_mode(ctrl_quadpid) != CTRL_MODE_STOP && --timeout);

    if (!timeout) {
        puts("Unexpected behavior");
        ctrl_set_mode(ctrl_quadpid, CTRL_MODE_STOP);
    }
}

/* Position calibration sequence */
static void ctrl_quadpid_pose_calib_seq(ctrl_t* ctrl_quadpid, pose_t pos)
{
    /* Speed order is fixed to maximum speed */
    polar_t speed_order = {
        .distance = MAX_SPEED,
        .angle = MAX_SPEED / 2,
    };

    /* Send the speed order and the position to reach to the controller */
    ctrl_set_pose_to_reach(ctrl_quadpid, pos);
    ctrl_set_speed_order(ctrl_quadpid, speed_order);

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
 * @param[in]   ctrl_quadpid    controller object.
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

static bool check_running_thread(void) {
    if (ctrl_quadpid_thread_command_pid == 0) {
        return FALSE;
    }
    kernel_pid_t status = thread_getstatus(ctrl_quadpid_thread_command_pid);
    if (status != (int)STATUS_NOT_FOUND && status != STATUS_STOPPED)
    {
        printf("Error: Previous command still running.\n");
        return TRUE;
    }
    return FALSE;
}

static void run_cmd_in_thread(void *(func)(void *arg)) {
    ctrl_quadpid_thread_command_pid = thread_create(
        quadpid_thread_stack,
        sizeof(quadpid_thread_stack),
        THREAD_PRIORITY_MAIN - 4, 0,
        func,
        NULL,
        "command thread"
    );
}

/**
 * @brief Linear speed characterization
 *
 * @param[in]   arg     not used
 */
static void *ctrl_quadpid_speed_thread_cmd_linear_speed(void *arg)
{
    (void)arg;
    /* Register speed order generation function to the controller */
    ctrl_register_speed_order_cb((ctrl_t *)ctrl_quadpid, func_impulse_on_speed_order);
    current_impulse_cfg = &impulse_cfg__tf_ident_linear;
    calib_seq_identify_robot_tf((ctrl_t *)ctrl_quadpid);
    ctrl_register_speed_order_cb((ctrl_t *)ctrl_quadpid, NULL);
    return 0;
}

/**
 * @brief Angular speed characterization command
 *
 * Run command in a dedicated thread
 *
 * @param[in]   argc    number of arguments
 * @param[in]   argv    arguments list
 *
 * @return              0 on success not 0 otherwise
 */
static int ctrl_quadpid_speed_cmd_linear_speed_cb(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    if(check_running_thread()) return EXIT_FAILURE;
    run_cmd_in_thread(ctrl_quadpid_speed_thread_cmd_linear_speed);
    return EXIT_SUCCESS;
}

/**
 * @brief Angular speed characterization
 *
 * @param[in]   arg     not used
 */
static void *ctrl_quadpid_speed_thread_cmd_angular_speed(void *arg)
{
    (void)arg;
    /* Register speed order generation function to the controller */
    ctrl_register_speed_order_cb((ctrl_t*)ctrl_quadpid, func_impulse_on_speed_order);
    current_impulse_cfg = &impulse_cfg__tf_ident_angular;
    calib_seq_identify_robot_tf((ctrl_t*)ctrl_quadpid);
    ctrl_register_speed_order_cb((ctrl_t*)ctrl_quadpid, NULL);
    return 0;
}

/**
 * @brief Angular speed characterization command
 *
 * Run command in a dedicated thread
 *
 * @param[in]   argc    number of arguments
 * @param[in]   argv    arguments list
 *
 * @return              0 on success not 0 otherwise
 */
static int ctrl_quadpid_speed_cmd_angular_speed_cb(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    if(check_running_thread()) return EXIT_FAILURE;
    run_cmd_in_thread(ctrl_quadpid_speed_thread_cmd_angular_speed);
    return EXIT_SUCCESS;
}

/**
 * @brief Linear speed PID test
 *
 * @param[in]   arg     not used
 */
static void *ctrl_quadpid_speed_thread_cmd_linear_pid(void *arg)
{
    (void)arg;
    ctrl_register_speed_order_cb((ctrl_t*)ctrl_quadpid, func_impulse_on_speed_order);
    current_impulse_cfg = &impulse_cfg__speed_pid_linear;
    calib_seq_speed_pid_only((ctrl_t*)ctrl_quadpid);
    ctrl_register_speed_order_cb((ctrl_t*)ctrl_quadpid, NULL);
    return 0;
}

/**
 * @brief Linear speed PID test command
 *
 * Run command in a dedicated thread
 *
 * @param[in]   argc    number of arguments
 * @param[in]   argv    arguments list
 *
 * @return              0 on success not 0 otherwise
 */
static int ctrl_quadpid_speed_cmd_linear_pid_cb(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    if(check_running_thread()) return EXIT_FAILURE;
    run_cmd_in_thread(ctrl_quadpid_speed_thread_cmd_linear_pid);
    return EXIT_SUCCESS;
}

/**
 * @brief Angular speed PID test
 *
 * @param[in]   arg     not used
 */
static void *ctrl_quadpid_speed_thread_cmd_angular_pid(void *arg)
{
    (void)arg;
    ctrl_register_speed_order_cb((ctrl_t*)ctrl_quadpid, func_impulse_on_speed_order);
    current_impulse_cfg = &impulse_cfg__speed_pid_angular;
    calib_seq_speed_pid_only((ctrl_t*)ctrl_quadpid);
    ctrl_register_speed_order_cb((ctrl_t*)ctrl_quadpid, NULL);
    return 0;
}

/**
 * @brief Angular speed PID test command
 *
 * Run command in a dedicated thread
 *
 * @param[in]   argc    number of arguments
 * @param[in]   argv    arguments list
 *
 * @return              0 on success not 0 otherwise
 */
static int ctrl_quadpid_speed_cmd_angular_pid_cb(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    if(check_running_thread()) return EXIT_FAILURE;
    run_cmd_in_thread(ctrl_quadpid_speed_thread_cmd_angular_pid);
    return EXIT_SUCCESS;
}

/**
 * @brief Set Linear Kp command
 *
 * @param[in]   argc    number of arguments
 * @param[in]   argv    arguments list
 *
 * @return              0 on success not 0 otherwise
 */
static int ctrl_quadpid_speed_cmd_set_linear_kp_cb(int argc, char **argv)
{
    /* Check arguments */
    if (argc != 2) {
        puts("Bad number of arguments!");
        return EXIT_FAILURE;
    }

    if(check_running_thread()) return EXIT_FAILURE;

    double kp = (double)atof(argv[1]);
    if (kp == 0.0) {
        puts("Bad value for Kp\n");
        return EXIT_FAILURE;
    }
    ctrl_quadpid->quadpid_params.linear_speed_pid.kp = kp;

    return EXIT_SUCCESS;
}

/**
 * @brief Set linear ki command
 *
 * @param[in]   argc    number of arguments
 * @param[in]   argv    arguments list
 *
 * @return              0 on success not 0 otherwise
 */
static int ctrl_quadpid_speed_cmd_set_linear_ki_cb(int argc, char **argv)
{
    /* Check arguments */
    if (argc != 2) {
        puts("Bad number of arguments!");
        return EXIT_FAILURE;
    }

    if(check_running_thread()) return EXIT_FAILURE;

    double ki = (double)atof(argv[1]);
    if (ki == 0.0) {
        puts("Bad value for Ki\n");
        return EXIT_FAILURE;
    }
    ctrl_quadpid->quadpid_params.linear_speed_pid.ki = ki;

    return EXIT_SUCCESS;
}

/**
 * @brief Set linear kd command
 *
 * @param[in]   argc    number of arguments
 * @param[in]   argv    arguments list
 *
 * @return              0 on success not 0 otherwise
 */
static int ctrl_quadpid_speed_cmd_set_linear_kd_cb(int argc, char **argv)
{
    /* Check arguments */
    if (argc != 2) {
        puts("Bad number of arguments!");
        return EXIT_FAILURE;
    }

    if(check_running_thread()) return EXIT_FAILURE;

    double kd = (double)atof(argv[1]);
    if (kd == 0.0) {
        puts("Bad value for Kd\n");
        return EXIT_FAILURE;
    }
    ctrl_quadpid->quadpid_params.linear_speed_pid.kd = kd;

    return EXIT_SUCCESS;
}

/**
 * @brief Set angular kp command
 *
 * @param[in]   argc    number of arguments
 * @param[in]   argv    arguments list
 *
 * @return              0 on success not 0 otherwise
 */
static int ctrl_quadpid_speed_cmd_set_angular_kp_cb(int argc, char **argv)
{
    /* Check arguments */
    if (argc != 2) {
        puts("Bad number of arguments!");
        return EXIT_FAILURE;
    }

    if(check_running_thread()) return EXIT_FAILURE;

    double kp = (double)atof(argv[1]);
    if (kp == 0.0) {
        puts("Bad value for Kp\n");
        return EXIT_FAILURE;
    }
    ctrl_quadpid->quadpid_params.angular_speed_pid.kp = kp;

    return EXIT_SUCCESS;
}

/**
 * @brief Set angular ki command
 *
 * @param[in]   argc    number of arguments
 * @param[in]   argv    arguments list
 *
 * @return              0 on success not 0 otherwise
 */
static int ctrl_quadpid_speed_cmd_set_angular_ki_cb(int argc, char **argv)
{
    /* Check arguments */
    if (argc != 2) {
        puts("Bad number of arguments!");
        return EXIT_FAILURE;
    }

    if(check_running_thread()) return EXIT_FAILURE;

    double ki = (double)atof(argv[1]);
    if (ki == 0.0) {
        puts("Bad value for Ki\n");
        return EXIT_FAILURE;
    }
    ctrl_quadpid->quadpid_params.angular_speed_pid.ki = ki;

    return EXIT_SUCCESS;
}

/**
 * @brief Set angular kd command
 *
 * @param[in]   argc    number of arguments
 * @param[in]   argv    arguments list
 *
 * @return              0 on success not 0 otherwise
 */
static int ctrl_quadpid_speed_cmd_set_angular_kd_cb(int argc, char **argv)
{
    /* Check arguments */
    if (argc != 2) {
        puts("Bad number of arguments!");
        return EXIT_FAILURE;
    }

    if(check_running_thread()) return EXIT_FAILURE;

    double kd = (double)atof(argv[1]);
    if (kd == 0.0) {
        puts("Bad value for Kd\n");
        return EXIT_FAILURE;
    }
    ctrl_quadpid->quadpid_params.angular_speed_pid.kd = kd;

    return EXIT_SUCCESS;
}

/**
 * @brief Reset all PID command
 *
 * @param[in]   argc    number of arguments
 * @param[in]   argv    arguments list
 *
 * @return              0 on success not 0 otherwise
 */
static int ctrl_quadpid_speed_cmd_reset_coef_cb(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    if(check_running_thread()) return EXIT_FAILURE;

    pid_reset_all(ctrl_quadpid);

    return EXIT_SUCCESS;
}

/**
 * @brief Global speed regulation command
 *
 * @param[in]   argc    number of arguments
 * @param[in]   argv    arguments list
 *
 * @return              0 on success not 0 otherwise
 */
static int ctrl_quadpid_speed_calib_cmd(int argc, char **argv)
{
    (void)argv;
    int ret = 0;

    /* Check arguments */
    if (argc > 1) {
        puts("Bad arguments number !");
        ret = -1;
        goto ctrl_quadpid_calib_servo_cmd_err;
    }

    /* Get the quadpid controller */
    ctrl_quadpid = pf_get_quadpid_ctrl();

    pf_init_shell_commands(&ctrl_quadpid_speed_shell_commands, quadpid_speed_name);

    shell_command_t ctrl_quadpid_speed_cmd_linear_speed = {
        "l", "Linear speed characterization",
        ctrl_quadpid_speed_cmd_linear_speed_cb
    };
    pf_add_shell_command(&ctrl_quadpid_speed_shell_commands, &ctrl_quadpid_speed_cmd_linear_speed);

    shell_command_t ctrl_quadpid_speed_cmd_angular_speed = {
        "a", "Angular speed characterization",
        ctrl_quadpid_speed_cmd_angular_speed_cb
    };
    pf_add_shell_command(&ctrl_quadpid_speed_shell_commands, &ctrl_quadpid_speed_cmd_angular_speed);

    shell_command_t ctrl_quadpid_speed_cmd_linear_pid = {
        "L", "Linear speed PID test",
        ctrl_quadpid_speed_cmd_linear_pid_cb
    };
    pf_add_shell_command(&ctrl_quadpid_speed_shell_commands, &ctrl_quadpid_speed_cmd_linear_pid);

    shell_command_t ctrl_quadpid_speed_cmd_angular_pid = {
        "A", "Angular speed PID test",
        ctrl_quadpid_speed_cmd_angular_pid_cb
    };
    pf_add_shell_command(&ctrl_quadpid_speed_shell_commands, &ctrl_quadpid_speed_cmd_angular_pid);

    shell_command_t ctrl_quadpid_speed_cmd_set_linear_kp = {
        "p", "Set linear Kp to <kp>",
        ctrl_quadpid_speed_cmd_set_linear_kp_cb
    };
    pf_add_shell_command(&ctrl_quadpid_speed_shell_commands, &ctrl_quadpid_speed_cmd_set_linear_kp);

    shell_command_t ctrl_quadpid_speed_cmd_set_linear_ki = {
        "i", "Set linear Ki to <ki>",
        ctrl_quadpid_speed_cmd_set_linear_ki_cb
    };
    pf_add_shell_command(&ctrl_quadpid_speed_shell_commands, &ctrl_quadpid_speed_cmd_set_linear_ki);

    shell_command_t ctrl_quadpid_speed_cmd_set_linear_kd = {
        "d", "Set linear Kd to <kd>",
        ctrl_quadpid_speed_cmd_set_linear_kd_cb
    };
    pf_add_shell_command(&ctrl_quadpid_speed_shell_commands, &ctrl_quadpid_speed_cmd_set_linear_kd);

    shell_command_t ctrl_quadpid_speed_cmd_set_angular_kp = {
        "P", "Set angular Kp to <kp>",
        ctrl_quadpid_speed_cmd_set_angular_kp_cb
    };
    pf_add_shell_command(&ctrl_quadpid_speed_shell_commands, &ctrl_quadpid_speed_cmd_set_angular_kp);

    shell_command_t ctrl_quadpid_speed_cmd_set_angular_ki = {
        "I", "Set angular Ki to <ki>",
        ctrl_quadpid_speed_cmd_set_angular_ki_cb
    };
    pf_add_shell_command(&ctrl_quadpid_speed_shell_commands, &ctrl_quadpid_speed_cmd_set_angular_ki);

    shell_command_t ctrl_quadpid_speed_cmd_set_angular_kd = {
        "D", "Set angular Kd to <kd>",
        ctrl_quadpid_speed_cmd_set_angular_kd_cb
    };
    pf_add_shell_command(&ctrl_quadpid_speed_shell_commands, &ctrl_quadpid_speed_cmd_set_angular_kd);

    shell_command_t ctrl_quadpid_speed_cmd_reset_coef = {
        "r", "Reset PID coefficients to (Kp = 1, Ki = 0, Kd = 0)",
        ctrl_quadpid_speed_cmd_reset_coef_cb
    };
    pf_add_shell_command(&ctrl_quadpid_speed_shell_commands, &ctrl_quadpid_speed_cmd_reset_coef);

    pf_add_shell_command(&ctrl_quadpid_speed_shell_commands, &cmd_exit_shell);

    /* Push new menu */
    pf_push_shell_commands(&ctrl_quadpid_speed_shell_commands);

ctrl_quadpid_calib_servo_cmd_err:
    return ret;
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


/**
 * @brief Encoders reset command
 *
 * @param[in]   argc    number of arguments
 * @param[in]   argv    arguments list
 *
 * @return              0 on success not 0 otherwise
 */
static int ctrl_quadpid_pose_cmd_reset_cb(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    if(check_running_thread()) return EXIT_FAILURE;

    encoder_reset();
    puts("<<<< RESET >>>>");

    return EXIT_SUCCESS;
}

static void *ctrl_quadpid_pose_thread_cmd_linear_kp(void *arg)
{
    (void)arg;
    ctrl_quadpid_pose_calib_seq((ctrl_t*)ctrl_quadpid, poses_calibration[pose_linear_index].pos);
    return 0;
}

static int ctrl_quadpid_pose_cmd_linear_kp_cb(int argc, char **argv)
{
    /* Check arguments */
    if (argc != 2) {
        puts("Bad number of arguments!");
        return EXIT_FAILURE;
    }

    double kp = (double)atof(argv[1]);
    if (kp == 0.0) {
        puts("Bad value for Kp\n");
        return EXIT_FAILURE;
    }

    if(check_running_thread()) return EXIT_FAILURE;

    encoder_reset();
    ctrl_set_pose_current((ctrl_t*)ctrl_quadpid, &poses_calibration[pose_linear_index].pos);
    pose_linear_index ^= 1;
    ctrl_quadpid->quadpid_params.linear_pose_pid.kp = kp;

    run_cmd_in_thread(ctrl_quadpid_pose_thread_cmd_linear_kp);

    return EXIT_SUCCESS;
}

static void *ctrl_quadpid_pose_thread_cmd_angular_kp(void *arg)
{
    (void)arg;
    ctrl_quadpid_pose_calib_seq((ctrl_t*)ctrl_quadpid, poses_calibration[pose_linear_index].pos);
    return 0;
}

static int ctrl_quadpid_pose_cmd_angular_kp_cb(int argc, char **argv)
{
    /* Check arguments */
    if (argc != 2) {
        puts("Bad number of arguments!");
        return EXIT_FAILURE;
    }

    double kp = (double)atof(argv[1]);
    if (kp == 0.0) {
        puts("Bad value for Kp\n");
        return EXIT_FAILURE;
    }

    if(check_running_thread()) return EXIT_FAILURE;

    encoder_reset();
    ctrl_set_pose_current((ctrl_t*)ctrl_quadpid, &poses_calibration[pose_linear_index].pos);
    pose_linear_index ^= 1;
    ctrl_quadpid->quadpid_params.angular_pose_pid.kp = kp;

    run_cmd_in_thread(ctrl_quadpid_pose_thread_cmd_angular_kp);

    return EXIT_SUCCESS;
}

/**
 * @brief Position calibration command
 *
 * @param[in]   argc    number of arguments
 * @param[in]   argv    arguments list
 *
 * @return              0 on success not 0 otherwise
 */
static int ctrl_quadpid_pose_calib_cmd(int argc, char **argv)
{
    (void)argv;
    int ret = 0;

    /* Check arguments */
    if (argc > 1) {
        puts("Bad arguments number !");
        ret = -1;
        goto ctrl_quadpid_calib_servo_cmd_err;
    }

    /* Get the quadpid controller */
    ctrl_quadpid = pf_get_quadpid_ctrl();

    /* Index on position to reach according to current angular one */

    /* Index on position to reach according to current linear one */
    pose_linear_index = 0;

    /* Automatic reverse */
    ctrl_set_allow_reverse((ctrl_t*)ctrl_quadpid, TRUE);

    pf_init_shell_commands(&ctrl_quadpid_pose_shell_commands, quadpid_pose_name);

    shell_command_t ctrl_quadpid_pose_cmd_reset = {
        "r", "Send reset",
        ctrl_quadpid_pose_cmd_reset_cb
    };
    pf_add_shell_command(&ctrl_quadpid_pose_shell_commands, &ctrl_quadpid_pose_cmd_reset);

    shell_command_t ctrl_quadpid_pose_cmd_linear_kp = {
        "a", "Speed linear Kp calibration to <kp>",
        ctrl_quadpid_pose_cmd_linear_kp_cb
    };
    pf_add_shell_command(&ctrl_quadpid_pose_shell_commands, &ctrl_quadpid_pose_cmd_linear_kp);

    shell_command_t ctrl_quadpid_pose_cmd_angular_kp = {
        "A", "Speed angular Kp calibration to <kp>",
        ctrl_quadpid_pose_cmd_angular_kp_cb
    };
    pf_add_shell_command(&ctrl_quadpid_pose_shell_commands, &ctrl_quadpid_pose_cmd_angular_kp);

    pf_add_shell_command(&ctrl_quadpid_pose_shell_commands, &cmd_exit_shell);

    /* Push new menu */
    pf_push_shell_commands(&ctrl_quadpid_pose_shell_commands);

ctrl_quadpid_calib_servo_cmd_err:
    return ret;
}

/**
 * @brief Initialize quapid controller calibration
 *
 * Register quapid controller calibration commands.
 */
void ctrl_quadpid_calib_init(void)
{
    /* Add speed calibration command */
    shell_command_t cmd_calib_speed = {
        "cs", "Speed PID coefficients tuning",
        ctrl_quadpid_speed_calib_cmd
    };
    pf_add_shell_command(&pf_shell_commands, &cmd_calib_speed);

    /* Add pose calibration command */
    shell_command_t cmd_calib_pose = {
        "cp", "Pose PID coefficients tuning",
        ctrl_quadpid_pose_calib_cmd
    };
    pf_add_shell_command(&pf_shell_commands, &cmd_calib_pose);
}
