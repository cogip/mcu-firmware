// RIOT includes
#include "periph/uart.h"
#include "ringbuffer.h"
#include "riot/chrono.hpp"
#include "riot/thread.hpp"
#include "thread.h"
#include "xtimer.h"

// Projet includes
#include "shell_menu/shell_menu.hpp"

#include "pingpong.pb.h"
#include "uartpb/UartProtobuf.hpp"

#define SENDER_PRIO        (THREAD_PRIORITY_MAIN - 1)

static kernel_pid_t sender_pid;
static char sender_stack[THREAD_STACKSIZE_MAIN];
bool suspend_sender = false;
void message_handler(const PB_Response &message);

cogip::uartpb::UartProtobuf<PB_Response, PB_Request, message_handler> *uartpb = nullptr;

// C++ types corresponding to Protobuf message types
class Pose {
public:
    Pose(double x=0.0, double y=0.0, double angle=0.0) : x(x), y(y), angle(angle) {};
    Pose(const PB_Pose &pose) : x(pose.x), y(pose.y), angle(pose.angle) {};

    void pb_copy(PB_Pose &pose) const {
        pose.x = x;
        pose.y = y;
        pose.angle = angle;
    }

    double x;
    double y;
    double angle;
};

enum class Color {
#define COLOR_MACRO(name, value) name = value,
#include "colors.inc"
#undef COLOR_MACRO
};

const char * const color_name[] = {
#define COLOR_MACRO(name, value) #name,
#include "colors.inc"
#undef COLOR_MACRO
};

void handle_response_hello(const PB_RespHello &hello)
{
    printf("<<== Hello response with number=%" PRId32 "\n\n", hello.number);
}

void handle_response_ping(const PB_RespPing &ping)
{
    printf("<<== Ping response with color=%s\n\n", color_name[ping.color]);
}

void handle_response_pong(const PB_RespPong &pong)
{
    Pose pose(pong.new_pose);
    printf(
        "<<== Pong response with pose={x=%.2lf, y=%.2lf, angle=%.2lf}\n\n",
        pose.x, pose.y, pose.angle
        );
}

void message_handler(const PB_Response &response)
{
    switch (response.which_response) {
        case PB_Response_hello_tag:
            handle_response_hello(response.response.hello);
            break;
        case PB_Response_ping_tag:
            handle_response_ping(response.response.ping);
            break;
        case PB_Response_pong_tag:
            handle_response_pong(response.response.pong);
            break;
        default:
            printf("Unknown request type: %d\n", response.which_response);
    }
}

static bool send_hello()
{
    PB_Request request = PB_Request_init_zero;
    request.which_request = PB_Request_hello_tag;
    PB_ReqHello &hello = request.request.hello;
    hello.number = std::rand();
    printf("==>> Hello request  with number=%" PRId32 "\n", hello.number);

    return uartpb->send_message(request);
}

static bool send_ping()
{
    PB_Request request = PB_Request_init_zero;
    request.which_request = PB_Request_ping_tag;
    PB_ReqPing &ping = request.request.ping;
    ping.color = (PB_ColorType)Color::RED;

    printf("==>> Ping request  with color=%s\n", color_name[ping.color]);

    return uartpb->send_message(request);
}

static bool send_pong()
{
    PB_Request request = PB_Request_init_zero;
    request.which_request = PB_Request_pong_tag;
    PB_ReqPong &pong = request.request.pong;
    pong.has_pose = true;
    Pose pose = {15, 30, 90};
    pose.pb_copy(pong.pose);

    printf(
        "==>> Pong request  with pose={x=%.2lf, y=%.2lf, angle=%.2lf}\n",
        pong.pose.x, pong.pose.y, pong.pose.angle
        );

    return uartpb->send_message(request);
}

static void *message_sender(void *arg)
{
    (void)arg;
    bool is_ping = true;

    while (1) {
        if (suspend_sender) {
            suspend_sender = false;
            thread_sleep();
        }

        riot::this_thread::sleep_for(std::chrono::seconds(2));

        if (is_ping) {
            send_ping();
        }
        else {
            send_pong();
        }
        is_ping = !is_ping;
    }

    return NULL;
}

static int cmd_hello(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    send_hello();

    return 0;
}

static int cmd_start(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    suspend_sender = false;
    thread_wakeup(sender_pid);

    return 0;
}

static int cmd_stop(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    suspend_sender = true;

    return 0;
}

int main(void)
{
    printf("\n== UART/NanoPB Example ==\n");

    cogip::shell::root_menu.push_back(
        new cogip::shell::Command("hello", "Send hello", cmd_hello));
    cogip::shell::root_menu.push_back(
        new cogip::shell::Command("start", "Start sender thread", cmd_start));
    cogip::shell::root_menu.push_back(
        new cogip::shell::Command("stop", "Stop sender thread", cmd_stop));

    uartpb = new cogip::uartpb::UartProtobuf<PB_Response, PB_Request, message_handler>(
        PB_Request_fields, PB_Response_fields, UART_DEV(1)
        );

    bool res = uartpb->connect();
    if (! res) {
        printf("UART initialization status: %d\n", res);
        exit(1);
    }

    uartpb->start_reader();

    sender_pid = thread_create(
        sender_stack, sizeof(sender_stack), SENDER_PRIO,
        THREAD_CREATE_SLEEPING, message_sender, NULL, "sender");

    // Start shell
    cogip::shell::start();

    return 0;
}
