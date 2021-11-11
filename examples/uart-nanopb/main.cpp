// RIOT includes
#include "periph/uart.h"
#include "ringbuffer.h"
#include "riot/chrono.hpp"
#include "riot/thread.hpp"
#include "thread.h"
#include "xtimer.h"

// Projet includes
#include "shell_menu/shell_menu.hpp"

#include <pb_encode.h>
#include <pb_decode.h>
#include "pingpong.pb.h"

#define READER_PRIO        (THREAD_PRIORITY_MAIN - 1)
#define SENDER_PRIO        (THREAD_PRIORITY_MAIN - 1)

static kernel_pid_t reader_pid, sender_pid;
static char reader_stack[THREAD_STACKSIZE_MAIN];
static char sender_stack[THREAD_STACKSIZE_MAIN];
uart_t uart_dev = UART_DEV(1);
uint32_t uart_speed = 230400U;
bool suspend_sender = false;

typedef struct {
    char rx_mem[PB_Response_size];
    ringbuffer_t rx_buf;
} uart_ctx_t;

static uart_ctx_t ctx;

static uint8_t msg_length_bytes_number = 4;
static uint8_t msg_length_bytes_received = 0;
static uint32_t msg_length = 0;
static uint32_t msg_bytes_received = 0;
static uint8_t input_buffer[PB_Response_size];
static uint8_t output_buffer[PB_Request_size];

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

static void uart_rx_cb(void *arg, uint8_t data)
{
    (void)arg;
    if (msg_length_bytes_received < msg_length_bytes_number) {
        ((uint8_t *)&msg_length)[msg_length_bytes_received++] = data;
        msg_bytes_received = 0;
        return;
    }

    if (msg_bytes_received < msg_length) {
        ringbuffer_add_one(&(ctx.rx_buf), data);
        msg_bytes_received++;
    }

    if (msg_bytes_received == msg_length) {
        msg_t msg;
        msg.content.value = msg_length;
        msg_send(&msg, reader_pid);
        msg_length_bytes_received = 0;
    }
}

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

static bool decode_message(uint32_t message_length, uint8_t *buffer)
{
    PB_Response response = PB_Response_init_zero;
    pb_istream_t stream = pb_istream_from_buffer(buffer, message_length);
    bool status = pb_decode(&stream, PB_Response_fields, &response);

    if (!status)
    {
        printf("Decoding failed: %s\n", PB_GET_ERROR(&stream));
        return false;
    }

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
    return true;
}

static void *message_reader(void *arg)
{
    (void)arg;
    msg_t msg;
    msg_t msg_queue[8];
    msg_init_queue(msg_queue, 8);

    while (1) {
        msg_receive(&msg);
        uint32_t message_length = (uint32_t)msg.content.value;
        ringbuffer_get(&(ctx.rx_buf), (char *)input_buffer, message_length);
        decode_message(message_length, input_buffer);
    }

    return NULL;
}

static void send_message(size_t message_length, uint8_t *message)
{
    uart_write(uart_dev, (uint8_t *)&message_length, 4);
    uart_write(uart_dev, message, message_length);
}

static bool send_request(const PB_Request &request)
{
    pb_ostream_t stream = pb_ostream_from_buffer(output_buffer, PB_Request_size);
    bool status = pb_encode(&stream, PB_Request_fields, &request);

    if (!status)
    {
        printf("Encoding failed: %s\n", PB_GET_ERROR(&stream));
        return false;
    }

    send_message(stream.bytes_written, output_buffer);
    return true;
}

static bool send_hello()
{
    PB_Request request = PB_Request_init_zero;
    request.which_request = PB_Request_hello_tag;
    PB_ReqHello &hello = request.request.hello;
    hello.number = std::rand();

    printf("==>> Hello request  with number=%" PRId32 "\n", hello.number);

    return send_request(request);
}

static bool send_ping()
{
    PB_Request request = PB_Request_init_zero;
    request.which_request = PB_Request_ping_tag;
    PB_ReqPing &ping = request.request.ping;
    ping.color = (PB_ColorType)Color::RED;

    printf("==>> Ping request  with color=%s\n", color_name[ping.color]);

    return send_request(request);
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

    return send_request(request);
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

        if (is_ping) {
            send_ping();
        }
        else {
            send_pong();
        }
        is_ping = !is_ping;

        riot::this_thread::sleep_for(std::chrono::seconds(2));
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

    int res = uart_init(uart_dev, uart_speed, uart_rx_cb, NULL);
    printf("UART initialization status: %d\n", res);

    cogip::shell::root_menu.push_back(
        new cogip::shell::Command("hello", "Send hello", cmd_hello));
    cogip::shell::root_menu.push_back(
        new cogip::shell::Command("start", "Start sender thread", cmd_start));
    cogip::shell::root_menu.push_back(
        new cogip::shell::Command("stop", "Stop sender thread", cmd_stop));

    ringbuffer_init(&(ctx.rx_buf), ctx.rx_mem, PB_Response_size);

    sender_pid = thread_create(
        sender_stack, sizeof(sender_stack), SENDER_PRIO,
        THREAD_CREATE_SLEEPING, message_sender, NULL, "sender");

    reader_pid = thread_create(
        reader_stack, sizeof(reader_stack), READER_PRIO,
        0, message_reader, NULL, "reader");

    // Start shell
    cogip::shell::start();

    return 0;
}
