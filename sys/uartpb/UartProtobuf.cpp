#include "uartpb/UartProtobuf.hpp"

// RIOT includes
#include "riot/chrono.hpp"
#include "riot/thread.hpp"
#include "xtimer.h"

#include "Errors.h"

namespace cogip {

namespace uartpb {

UartProtobuf::UartProtobuf(
    message_handler_t message_handler,
    uart_t uart_dev, uint32_t uart_speed) :
    uart_dev_(uart_dev), uart_speed_(uart_speed)
{
    message_handler_ = message_handler;
    ringbuffer_init(&rx_buf_, rx_mem_, UART_BUFFER_SIZE);
}

bool UartProtobuf::connect()
{
    int res = uart_init(uart_dev_, uart_speed_, uart_rx_cb_wrapper, (void *)this);
    return res == UART_OK;
}

void UartProtobuf::start_reader()
{
    reader_pid_ = thread_create(
        reader_stack_, sizeof(reader_stack_), UARTPB_READER_PRIO,
        0, message_reader_wrapper, (void *)this, "Protobuf reader");
}

void UartProtobuf::uart_rx_cb(uint8_t data)
{
    // Read message type.
    if (msg_type_ == UINT8_MAX) {
        msg_type_ = data;
        msg_length_bytes_received_ = 0;
        return;
    }

    // Read the size of the message.
    if (msg_length_bytes_received_ < msg_length_bytes_number_) {
        ((uint8_t *)&msg_length_)[msg_length_bytes_received_++] = data;
        msg_bytes_received_ = 0;
        if (msg_length_bytes_received_ == msg_length_bytes_number_ && msg_length_ == 0) {
            // Special case where the message content is empty.
            // Directly send the message with its type and no content.
            msg_t msg;
            msg.type = msg_type_;
            msg.content.value = msg_length_;
            msg_send(&msg, reader_pid_);
            msg_type_ = UINT8_MAX;
        }
        return;
    }

    // If the size is received, read the message.
    if (msg_bytes_received_ < msg_length_) {
        ringbuffer_add_one(&rx_buf_, data);
        msg_bytes_received_++;
    }

    // If the message is received, send it to the decoding thread.
    if (msg_bytes_received_ == msg_length_) {
        msg_t msg;
        msg.type = msg_type_;
        msg.content.value = msg_length_;
        msg_send(&msg, reader_pid_);
        msg_type_ = UINT8_MAX;
    }
}

void UartProtobuf::message_reader()
{
    msg_t msg;
    msg_t msg_queue[8];
    msg_init_queue(msg_queue, 8);

    while (1) {
        msg_receive(&msg);
        uint8_t message_type = (uint8_t)msg.type;
        uint32_t message_length = (uint32_t)msg.content.value;
        assert(message_length <= UARTPB_INPUT_MESSAGE_LENGTH_MAX);
        read_buffer_.clear();
        for (size_t i = 0; i < message_length; i++) {
            uint8_t c = ringbuffer_get_one(&rx_buf_);
            read_buffer_.push(c);
        }
        if (message_handler_) {
            message_handler_(message_type, read_buffer_);
        }
    }
}

bool UartProtobuf::send_message(uint8_t message_type)
{
    bool success = true;
    mutex_.lock();

    write_buffer_.clear();
    write_buffer_.set_message_type(message_type);
    size_t base64_size = write_buffer_.base64_encode();
    if (base64_size == 0) {
        puts("Failed to base64 encode Protobuf serialized message.");
        success = false;
    }
    else {
        uart_write(uart_dev_, write_buffer_.get_base64_data(), base64_size+1);
    }

    mutex_.unlock();

    return success;
}

bool UartProtobuf::send_message(uint8_t message_type, const EmbeddedProto::MessageInterface &message)
{
    bool success = true;
    mutex_.lock();
    write_buffer_.clear();
    auto serialization_status = message.serialize(write_buffer_);
    if(EmbeddedProto::Error::NO_ERRORS != serialization_status) {
        puts("Failed to serialize Protobuf message.");
        success = false;
    }
    else {
        write_buffer_.set_message_type(message_type);
        size_t base64_size = write_buffer_.base64_encode();
        if (base64_size == 0) {
            puts("Failed to base64 encode Protobuf serialized message.");
            success = false;
        }
        else {
            uart_write(uart_dev_, write_buffer_.get_base64_data(), base64_size+1);
        }

    }

    mutex_.unlock();

    return success;
}

} // namespace uartpb

} // namespace cogip
