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

cogip::uartpb::UartProtobuf *uartpb = nullptr;

constexpr cogip::uartpb::uuid_t reset_uuid = 3351980141;
constexpr cogip::uartpb::uuid_t req_hello_uuid = 3938291130;
constexpr cogip::uartpb::uuid_t req_ping_uuid = 2537089183;
constexpr cogip::uartpb::uuid_t req_pong_uuid = 3650317449;
constexpr cogip::uartpb::uuid_t resp_hello_uuid = 4187249687;
constexpr cogip::uartpb::uuid_t resp_ping_uuid = 4288740491;
constexpr cogip::uartpb::uuid_t resp_pong_uuid = 2687718320;

static PB_RespHello resp_hello;
static PB_RespPing resp_ping;
static PB_RespPong resp_pong;

static PB_ReqHello<64> req_hello;
static PB_ReqPing req_ping;
static PB_ReqPong req_pong;

static void handle_response_hello(cogip::uartpb::ReadBuffer *buffer)
{
    resp_hello.deserialize(*buffer);
    std::cout << "<<== Hello response with number=" << resp_hello.get_number() << std::endl;
}

static void handle_response_ping(cogip::uartpb::ReadBuffer *buffer)
{
    resp_ping.deserialize(*buffer);
    std::cout << "<<== Ping response with color="<< get_color_name((cogip::cogip_defs::Color)resp_ping.get_color()) << std::endl;
}

static void handle_response_pong(cogip::uartpb::ReadBuffer *buffer)
{
    resp_pong.deserialize(*buffer);
    const cogip::cogip_defs::Pose &pose = resp_pong.get_new_pose();
    std::cout
        << "<<== Pong response with pose={x=" << pose.x()
        << ", y=" << pose.y() << ", angle=" << pose.O() << "}"
        << std::endl;
}

static void send_hello()
{
    req_hello.set_number(std::rand());
    req_hello.mutable_message() = "hellohello";
    std::cout
        << "==>> Hello request  with number=" << (int32_t)req_hello.get_number()
        << " and message='" << req_hello.get_message().get_const() << "'"
        << std::endl;

    uartpb->send_message(req_hello_uuid, &req_hello);
}

static void send_ping()
{
    req_ping.set_color((PB_Color)cogip::cogip_defs::Color::RED);

    std::cout << "==>> Ping request  with color=" << get_color_name((cogip::cogip_defs::Color)req_ping.get_color()) << std::endl;

    uartpb->send_message(req_ping_uuid, &req_ping);
}

static void send_pong()
{
    cogip::cogip_defs::Pose pose = {15, 30, 90};
    pose.pb_copy(req_pong.mutable_pose());

    std::cout
        << "==>> Pong request  with pose={x=" << req_pong.get_pose().get_x()
        << ", y=" << req_pong.get_pose().get_y()
        << ", angle=" << req_pong.get_pose().get_O() << "}"
        << std::endl;

    uartpb->send_message(req_pong_uuid, &req_pong);
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

    std::cout << "Sub-menu command" << std::endl;

    return 0;
}

int main(void)
{
    std::cout << std::endl << "== UART/EmbeddedProto Example ==" << std::endl;

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

    uartpb = new cogip::uartpb::UartProtobuf(UART_DEV(1));

    uartpb->register_message_handler(resp_hello_uuid, handle_response_hello);
    uartpb->register_message_handler(resp_ping_uuid, handle_response_ping);
    uartpb->register_message_handler(resp_pong_uuid, handle_response_pong);

    bool res = uartpb->connect();
    if (! res) {
        std::cout << "UART initialization status: " << res << std::endl;
        exit(1);
    }

    uartpb->start_reader();

    sender_pid = thread_create(
        sender_stack, sizeof(sender_stack), SENDER_PRIO,
        THREAD_CREATE_SLEEPING, message_sender, NULL, "sender");

    uartpb->send_message(reset_uuid);

    // Start shell
    cogip::shell::register_uartpb(uartpb);
    cogip::shell::start();

    return 0;
}
