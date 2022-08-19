/* RIOT includes */
#include "log.h"
#include "periph/qdec.h"

/* Project includes */
#include "app.hpp"
#include "board.h"
#include "obstacles/obstacles.hpp"
#include "obstacles/Obstacle.hpp"
#include "planners/astar/AstarPlanner.hpp"
#include "platform.hpp"
#include "avoidance.hpp"
#include "path/Path.hpp"
#include "shell_menu/shell_menu.hpp"
#include "uartpb/UartProtobuf.hpp"
#include "uartpb/ReadBuffer.hpp"
#include "utils.hpp"

/* Platform includes */
#include "lidar_utils.hpp"
#include "lidar_obstacles.hpp"
#include "trace_utils.hpp"

#include "PB_Command.hpp"
#include "PB_State.hpp"

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
        .min_angle_for_target_orientation = 0,
        .regul = CTRL_REGUL_IDLE
    }
};

/* Thread stacks */
char controller_thread_stack[THREAD_STACKSIZE_LARGE];
char countdown_thread_stack[THREAD_STACKSIZE_DEFAULT];

static bool copilot_connected = false;
PB_State<AVOIDANCE_GRAPH_MAX_VERTICES, OBSTACLES_MAX_NUMBER, OBSTACLE_BOUNDING_BOX_VERTICES> pb_state;

cogip::uartpb::UartProtobuf uartpb(UART_DEV(1));
cogip::wizard::Wizard wizard(uartpb);

// Define uartpb uuids
constexpr cogip::uartpb::uuid_t reset_uuid = 3351980141;
constexpr cogip::uartpb::uuid_t state_uuid = 3422642571;
constexpr cogip::uartpb::uuid_t copilot_connected_uuid = 1132911482;
constexpr cogip::uartpb::uuid_t copilot_disconnected_uuid = 1412808668;

bool pf_trace_on(void)
{
    return copilot_connected;
}

void pf_set_copilot_connected(bool connected)
{
    copilot_connected = connected;
}

void pf_print_state(void)
{
    ctrl_t *ctrl = (ctrl_t *)&ctrl_quadpid;

    COGIP_DEBUG_COUT(
        "{"
            << "\"mode\":" << ctrl->control.current_mode << ","
            << "\"pose_current\":{"
                << "\"O\":" << ctrl->control.pose_current.O()
                << "\"x\":" << ctrl->control.pose_current.x()
                << ",\"y\":" << ctrl->control.pose_current.y()
            << "},"
            << "\"pose_order\":{"
                << "\"O\":" << ctrl->control.pose_order.O()
                << "\"x\":" << ctrl->control.pose_order.x()
                << ",\"y\":" << ctrl->control.pose_order.y()
            << "},"
            << "\"cycle\":" << ctrl->control.current_cycle << ","
            << "\"speed_current\":{\"distance\":" << ctrl->control.speed_current.distance()
            << ",\"angle\":" << ctrl->control.speed_current.angle() << "},"
            << ",\"path\":"
    );
    avoidance_print_path();

    COGIP_DEBUG_COUT(",\"obstacles\":");
    cogip::obstacles::print_all_json();

    COGIP_DEBUG_COUT("}");
}

void pf_send_pb_state(void)
{
    ctrl_t *ctrl = (ctrl_t *)&ctrl_quadpid;
    pb_state.clear();
    pb_state.set_mode((PB_Mode)ctrl->control.current_mode);
    ctrl->control.pose_current.pb_copy(pb_state.mutable_pose_current());
    ctrl->control.pose_order.pb_copy(pb_state.mutable_pose_order());
    pb_state.mutable_cycle() = ctrl->control.current_cycle;
    ctrl->control.speed_current.pb_copy(pb_state.mutable_speed_current());
    ctrl->control.speed_order.pb_copy(pb_state.mutable_speed_order());
    avoidance_pb_copy_path(pb_state.mutable_path());
    cogip::obstacles::pb_copy(pb_state.mutable_obstacles());

    uartpb.send_message(state_uuid, &pb_state);
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

cogip::planners::Planner & pf_get_planner(void)
{
  static cogip::planners::AstarPlanner planner((ctrl_t *)&ctrl_quadpid, app_get_path());
  return planner;
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
    int32_t left_speed = qdec_read_and_reset(MOTOR_LEFT) * QDEC_LEFT_POLARITY;
    int32_t right_speed = qdec_read_and_reset(MOTOR_RIGHT) * QDEC_RIGHT_POLARITY;

    /* update speed */
    robot_speed.set_distance(((right_speed + left_speed) / 2.0) / PULSE_PER_MM);
    robot_speed.set_angle((right_speed - left_speed) / PULSE_PER_DEGREE);

    return 0;
}

void encoder_reset(void)
{
    qdec_read_and_reset(MOTOR_LEFT);
    qdec_read_and_reset(MOTOR_RIGHT);
}

void motor_drive(const cogip::cogip_defs::Polar &command)
{
    int16_t right_command = (int16_t) (command.distance() + command.angle());
    int16_t left_command = (int16_t) (command.distance() - command.angle());

    motor_set(MOTOR_DRIVER_DEV(MOTOR_LEFT), 0, left_command);
    motor_set(MOTOR_DRIVER_DEV(MOTOR_RIGHT), 0, right_command);
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

cogip::uartpb::UartProtobuf & pf_get_uartpb()
{
    return uartpb;
}

cogip::wizard::Wizard & pf_get_wizard()
{
    return wizard;
}

void handle_copilot_connected(cogip::uartpb::ReadBuffer &)
{
    pf_set_copilot_connected(true);
    std::cout << "Copilot connected" << std::endl;
    if (cogip::shell::current_menu) {
        cogip::shell::current_menu->send_pb_message();
    }
}

void handle_copilot_disconnected(cogip::uartpb::ReadBuffer &)
{
    pf_set_copilot_connected(false);
    std::cout << "Copilot disconnected" << std::endl;
}

void pf_init(void)
{
    /* Initialize UARTPB */
    bool uartpb_res = uartpb.connect();
    if (! uartpb_res) {
        COGIP_DEBUG_CERR("UART initialization failed, error: " << uartpb_res);
    }
    else {
        cogip::shell::register_uartpb(&uartpb);
        uartpb.register_message_handler(
            copilot_connected_uuid,
            cogip::uartpb::message_handler_t::create<handle_copilot_connected>()
            );
        uartpb.register_message_handler(
            copilot_disconnected_uuid,
            cogip::uartpb::message_handler_t::create<handle_copilot_disconnected>()
            );
        uartpb.start_reader();
        uartpb.send_message(reset_uuid);
    }

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
    int error = qdec_init(QDEC_DEV(MOTOR_LEFT), QDEC_MODE, NULL, NULL);
    if (error) {
        printf("QDEC %u not initialized, error=%d !!!\n", MOTOR_LEFT, error);
    }
    error = qdec_init(QDEC_DEV(MOTOR_RIGHT), QDEC_MODE, NULL, NULL);
    if (error) {
        printf("QDEC %u not initialized, error=%d !!!\n", MOTOR_RIGHT, error);
    }

    /* Init odometry */
    odometry_setup(WHEELS_DISTANCE / PULSE_PER_MM);

    gpio_clear(GPIO_DEBUG_LED);

    /*ctrl_set_anti_blocking_on(pf_get_ctrl(), TRUE);*/

#ifdef MODULE_SHELL_QUADPID
    ctrl_quadpid_shell_init(&ctrl_quadpid);
#endif /* MODULE_SHELL_QUADPID */
}

void pf_init_tasks(void)
{
    ctrl_t *controller = pf_get_ctrl();

    lidar_start(LIDAR_MAX_DISTANCE, LIDAR_MINIMUN_INTENSITY);

    obstacle_updater_start(ctrl_get_pose_current(controller));

    /* Create controller thread */
    thread_create(
        controller_thread_stack,
        sizeof(controller_thread_stack),
        THREAD_PRIORITY_MAIN - 4, 0,
        task_ctrl_update,
        (void *)controller,
        "motion control"
        );

    pf_get_planner().start_thread();

    trace_start();
}
