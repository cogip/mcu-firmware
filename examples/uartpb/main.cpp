// RIOT includes
#include "periph/uart.h"
#include "ringbuffer.h"
#include "riot/chrono.hpp"
#include "riot/thread.hpp"
#include "thread.h"
#include "xtimer.h"

// Projet includes
#include "shell_menu/shell_menu.hpp"

#include "Color.hpp"
#include "cogip_defs/Pose.hpp"

#include "PB_ReqHello.hpp"
#include "PB_ReqPing.hpp"
#include "PB_ReqPong.hpp"
#include "PB_RespHello.hpp"
#include "PB_RespPing.hpp"
#include "PB_RespPong.hpp"

#include "uartpb/UartProtobuf.hpp"
#include "uartpb/ReadBuffer.hpp"

#define SENDER_PRIO        (THREAD_PRIORITY_MAIN - 1)

static kernel_pid_t sender_pid;
static char sender_stack[THREAD_STACKSIZE_MAIN];
bool suspend_sender = false;

enum MessageType {
    MSG_MENU = 0,
    MSG_RESET = 1,
    MSG_HELLO = 2,
    MSG_PING = 3,
    MSG_PONG = 4
};

cogip::uartpb::UartProtobuf *uartpb = nullptr;

static void handle_response_hello(const cogip::embedded_proto_uart::PB_RespHello *hello)
{
    printf("<<== Hello response with number=%" PRId32 "\n\n", (int32_t)hello->get_number());
}

static void handle_response_ping(const cogip::embedded_proto_uart::PB_RespPing *ping)
{
    printf("<<== Ping response with color=%s\n\n", get_color_name((cogip::cogip_defs::Color)ping->get_color()));
}

static void handle_response_pong(const cogip::embedded_proto_uart::PB_RespPong *pong)
{
    cogip::cogip_defs::Pose pose(pong->get_new_pose());
    printf(
        "<<== Pong response with pose={x=%.2lf, y=%.2lf, angle=%.2lf}\n\n",
        pose.x(), pose.y(), pose.O()
        );
}

void message_handler(uint8_t message_type, cogip::uartpb::ReadBuffer &buffer)
{
    typedef void (*response_handler_t)(const EmbeddedProto::MessageInterface *);
    EmbeddedProto::MessageInterface *message = nullptr;
    response_handler_t response_handler = nullptr;
    switch (message_type) {
        case MSG_HELLO:
            message = new cogip::embedded_proto_uart::PB_RespHello();
            response_handler = (response_handler_t)handle_response_hello;
            break;
        case MSG_PING:
            message = new cogip::embedded_proto_uart::PB_RespPing();
            response_handler = (response_handler_t)handle_response_ping;
            break;
        case MSG_PONG:
            message = new cogip::embedded_proto_uart::PB_RespPong();
            response_handler = (response_handler_t)handle_response_pong;
            break;
        default:
            printf("Unknown response type: %u\n", message_type);
            return;
    }

    message->deserialize(buffer);
    response_handler(message);
}

static bool send_hello()
{
    cogip::embedded_proto_uart::PB_ReqHello<64> hello;
    hello.set_number(std::rand());
    hello.mutable_message() = "hellohello";
    printf(
        "==>> Hello request  with number=%" PRIi32 " and message='%s'\n",
        (int32_t)hello.get_number(), (const char *)hello.get_message().get_const()
        );

    return uartpb->send_message((uint8_t)MSG_HELLO, hello);
}

static bool send_ping()
{
    cogip::embedded_proto_uart::PB_ReqPing ping;
    ping.set_color((cogip::cogip_defs::PB_Color)cogip::cogip_defs::Color::RED);

    printf("==>> Ping request  with color=%s\n", get_color_name((cogip::cogip_defs::Color)ping.get_color()));

    return uartpb->send_message((uint8_t)MSG_PING, ping);
}

static bool send_pong()
{
    cogip::embedded_proto_uart::PB_ReqPong pong;
    cogip::cogip_defs::Pose pose = {15, 30, 90};
    pose.pb_copy(pong.mutable_pose());

    printf(
        "==>> Pong request  with pose={x=%.2lf, y=%.2lf, angle=%.2lf}\n",
        (double)pong.get_pose().get_x(), (double)pong.get_pose().get_y(), (double)pong.get_pose().get_O()
        );

    return uartpb->send_message((uint8_t)MSG_PONG, pong);
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

static int cmd_sub(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    printf("Sub-menu command\n");

    return 0;
}

int main(void)
{
    printf("\n== UART/EmbeddedProto Example ==\n");

    cogip::shell::root_menu.push_back(
        new cogip::shell::Command("hello", "Send hello", cmd_hello));
    cogip::shell::root_menu.push_back(
        new cogip::shell::Command("start", "Start sender thread", cmd_start));
    cogip::shell::root_menu.push_back(
        new cogip::shell::Command("stop", "Stop sender thread", cmd_stop));

    // Add a sub-menu
    cogip::shell::Menu *menu = new cogip::shell::Menu(
        "Sub-menu", "sub", &cogip::shell::root_menu);
    menu->push_back(new cogip::shell::Command("cmd", "Sub-menu command", cmd_sub));

    uartpb = new cogip::uartpb::UartProtobuf(
        message_handler,
        UART_DEV(1)
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

    uartpb->send_message((uint8_t)MSG_RESET);

    // Start shell
    cogip::shell::register_uartpb(uartpb);
    cogip::shell::start();

    return 0;
}
