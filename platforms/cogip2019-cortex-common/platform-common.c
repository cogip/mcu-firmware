/* System includes */
#include <thread.h>

/* RIOT includes */
#define ENABLE_DEBUG        (0)
#include "debug.h"
#include "log.h"
#include "shell.h"
#include "xtimer.h"

/* Project includes */
#include "avoidance.h"
#include "planner.h"
#include "platform.h"
#include "platform-common.h"

/* Radio includes */
#if defined(MODULE_CC110X)
#include "cc110x.h"
#include "cc110x-spi.h"
#include "cc110x-defines.h"
#endif

/* Controller */
static ctrl_quadpid_t ctrl_quadpid =
{
    .conf = &ctrl_quadpid_conf,
    .pf_conf = &ctrl_pf_quadpid_conf,
    .quadpid_params = ctrl_quadpid_params,
};

/* Thread stacks */
char controller_thread_stack[THREAD_STACKSIZE_LARGE];
char countdown_thread_stack[THREAD_STACKSIZE_DEFAULT];
char planner_thread_stack[THREAD_STACKSIZE_LARGE];
char start_shell_thread_stack[THREAD_STACKSIZE_LARGE];
char radio_thread_stack[THREAD_STACKSIZE_DEFAULT];

/* Shell command array */
static shell_command_t shell_commands[NB_SHELL_COMMANDS];

void pf_add_shell_command(shell_command_t *command)
{
    static uint8_t command_id = 0;

    assert(command_id < NB_SHELL_COMMANDS);

    shell_commands[command_id++] = *command;
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

    motor_set(0, HBRIDGE_MOTOR_LEFT, left_command);
    motor_set(0, HBRIDGE_MOTOR_RIGHT, right_command);
}

/* Init all known fixed obstacles on map */
void pf_fixed_obstacles_init(void)
{
    polygon_t polygon;
    uint8_t nb_vertices;

    /* Accelerator */
    polygon.count = 0;
    nb_vertices = 4;
    if (nb_vertices < POLY_MAX_POINTS) {
        polygon.points[polygon.count++] = (pose_t){.x = -1000, .y = 0 };
        polygon.points[polygon.count++] = (pose_t){.x =  1000, .y = 0 };
        polygon.points[polygon.count++] = (pose_t){.x =  1000, .y = 70 + ROBOT_MARGIN};
        polygon.points[polygon.count++] = (pose_t){.x = -1000, .y = 70 + ROBOT_MARGIN};
        add_polygon(&polygon);
    }

    /* Balance */
    polygon.count = 0;
    nb_vertices = 4;
    if (nb_vertices < POLY_MAX_POINTS) {
        polygon.points[polygon.count++] = (pose_t){.x = -1050 - ROBOT_MARGIN, .y = 1540 - ROBOT_MARGIN};
        polygon.points[polygon.count++] = (pose_t){.x =  1050 + ROBOT_MARGIN, .y = 1540 - ROBOT_MARGIN};
        polygon.points[polygon.count++] = (pose_t){.x =  1050 + ROBOT_MARGIN, .y = 2000};
        polygon.points[polygon.count++] = (pose_t){.x = -1050 - ROBOT_MARGIN, .y = 2000};
        add_polygon(&polygon);
    }

    /* Central stuff */
    polygon.count = 0;
    nb_vertices = 4;
    if (nb_vertices < POLY_MAX_POINTS) {
        polygon.points[polygon.count++] = (pose_t){.x = -20 - ROBOT_MARGIN, .y = 1350 - ROBOT_MARGIN};
        polygon.points[polygon.count++] = (pose_t){.x =  20 + ROBOT_MARGIN, .y = 1350 - ROBOT_MARGIN};
        polygon.points[polygon.count++] = (pose_t){.x =  20 + ROBOT_MARGIN, .y = 2000};
        polygon.points[polygon.count++] = (pose_t){.x = -20 - ROBOT_MARGIN, .y = 2000};
        add_polygon(&polygon);
    }
}

#if defined(MODULE_CC110X)

#define FSIZE 32

static uint32_t nb_pkt_tot = 0;

#define CAMP_LEFT 1
#define CAMP_RIGHT 0

static const cc110x_params_t cc110x_params[] = {
    CC110X_PARAMS
};

static cc110x_t pf_cc1101_device;

const char cc110x_cortex_pa_table[8] = {
    0x60,
    0x60,
    0x60,
    0x60,
    0x60,
    0x60,
    0x60,
    0x60
};

const char cc110x_cortex_base_freq[3] = { 0x21, 0x62, 0x76 };

static void emitter_loop(void)
{
    cc110x_t *dev = &pf_cc1101_device;

    uint8_t pkt[FSIZE+1] = {FSIZE};
    memset(pkt+1, 0, FSIZE);

    pkt[0] = FSIZE;

    for(uint8_t i = 2; i < FSIZE+1; i++)
        pkt[i] = i + 1;

    cc110x_write_reg(dev, CC110X_IOCFG2,
                     CC110X_GDO_LOW_ON_TX_FIFO_BELOW_THRESHOLD);

    for (uint8_t quit = 0; !quit;) {

        quit = 1;
        printf("Will send a pkt (%ld)\n", ++nb_pkt_tot);

        /* Put CC110x in IDLE mode to flush the FIFO */
        cc110x_strobe(dev, CC110X_SIDLE);

        printf("Flush TX FIFO to be sure it is empty\n");
        /* Flush TX FIFO to be sure it is empty */
        cc110x_strobe(dev, CC110X_SFTX);

        {
            static uint8_t direction = 0;
            static uint32_t motor_time_sec = 120 * US_PER_MS; /* 120 sec */

            for (uint8_t i = 2; i < FSIZE+1; i++)
                pkt[i] = direction ? 0x11 : 0xee;

            if (1) {
                pkt[2] = 0x55;
                pkt[3] = 0xaa;
                //pkt[4] = CAMP_LEFT;
                ////pkt[4] = CAMP_RIGHT;
                pkt[4] = !pf_is_camp_left() ? CAMP_LEFT : CAMP_RIGHT;

                /* climb time in microseconds */
                pkt[5] = (uint8_t) ((motor_time_sec & 0x000000ff) >> 0);
                pkt[6] = (uint8_t) ((motor_time_sec & 0x0000ff00) >> 8);
                pkt[7] = (uint8_t) ((motor_time_sec & 0x00ff0000) >> 16);
                pkt[8] = (uint8_t) ((motor_time_sec & 0xff000000) >> 24);
            }

            direction = !direction;
        }

        printf("Write packet into TX FIFO\n");
        /* Write packet into TX FIFO */
        cc110x_writeburst_reg(dev, CC110X_TXFIFO, (char *)pkt, FSIZE+1);

        printf("Before TX mode : %d bytes to send\n", cc110x_read_status(dev, CC110X_TXBYTES));
        /* Switch to TX mode */
        cc110x_strobe(dev, CC110X_STX);

        /* Wait that the TX FIFO is empty */
        uint8_t nb_wait = 0;
        while (cc110x_read_status(dev, CC110X_TXBYTES) & 0x7f) {
            xtimer_usleep(15);
            nb_wait++;
        }
        printf("... Waited %d us (%d)\n", nb_wait * 15, cc110x_read_status(dev, CC110X_TXBYTES));

        /* Print sent packet content */
        for (uint8_t i = 0; i < FSIZE + 1; i++) {
            printf("%02x ", pkt[i]);
        }
        printf("\n");
    }
}

static void emitter_init(void)
{
    cc110x_t *dev = &pf_cc1101_device;

    uint8_t bus = SPI_DEV(0);
    cc110x_setup(dev, &cc110x_params[bus]);
    gpio_init_af(spi_config[bus].mosi_pin, GPIO_AF7);

    cc110x_set_channel(dev, 0);

    //cc110x_write_reg(dev, CC110X_SYNC1, 0xB5);
    //cc110x_write_reg(dev, CC110X_SYNC0, 0x47);

    cc110x_write_reg(dev, CC110X_ADDR, 0x04);

    printf("CC1101_TEST0 = 0x%x\n", cc110x_read_reg(dev, CC110X_TEST0));
    printf("CC1101_TEST1 = 0x%x\n", cc110x_read_reg(dev, CC110X_TEST1));
    printf("CC1101_TEST2 = 0x%x\n", cc110x_read_reg(dev, CC110X_TEST2));

    printf("CC1101_PARTNUM = 0x%x\n", cc110x_read_status(dev, CC110X_PARTNUM));

    printf("CC1101_VERSION = 0x%x\n", cc110x_read_status(dev, CC110X_VERSION));
}
#else
#define emitter_init()
#define emitter_loop()
#endif /* defined(MODULE_CC110X) */

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
                countdown--);
        }
        xtimer_periodic_wakeup(&loop_start_time, US_PER_SEC);
    }

    return NULL;
}

void *task_radio(void *arg)
{
    (void)arg;

    LOG_INFO("Radio thread started\n");
    emitter_init();

    ///* Wait for start switch */
    //while(!pf_is_game_launched())
    //    ;

    for (;;) {
        //LOG_INFO("Radio thread started\n");
        xtimer_ticks32_t loop_start_time = xtimer_now();
        LOG_INFO(" -------------------- Loop radio\n");

        emitter_loop();

        xtimer_periodic_wakeup(&loop_start_time, 100 * US_PER_MS);
    }
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

    /* Create thread that up a flag on key pressed to start a shell instead of
       planner below */
    kernel_pid_t start_shell_pid = thread_create(start_shell_thread_stack,
                  sizeof(start_shell_thread_stack),
                  THREAD_PRIORITY_MAIN + 1, 0,
                  pf_task_start_shell, &start_shell, "shell");

    LOG_INFO("Press Enter to enter calibration mode...\n");

    /* Wait for Enter key pressed or countdown */
    while ((!start_shell) && (countdown > 0)) {
        xtimer_ticks32_t loop_start_time = xtimer_now();
        LOG_INFO("%d left...\n", countdown--);
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
    /* Create radio thread */
    thread_create(radio_thread_stack,
                  sizeof(radio_thread_stack),
                  THREAD_PRIORITY_MAIN + 1, 0,
                  task_radio,
                  (void*)NULL,
                  "radio control");

    /* If Enter was pressed, start shell */
    if (start_shell) {
        /* Define buffer to be used by the shell */
        char line_buf[SHELL_DEFAULT_BUFSIZE];
        /* Add end NULL entry to shell_command */
        shell_command_t null_command = { NULL, NULL, NULL };
        pf_add_shell_command(&null_command);
        /* Start shell */
        DEBUG("platform: Start shell\n");
        shell_run(shell_commands, line_buf, SHELL_DEFAULT_BUFSIZE);
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
