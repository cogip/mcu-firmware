#include "uartpb/UartProtobuf.hpp"

// System includes
#include <iostream>

// RIOT includes

#include "Errors.h"

namespace cogip {

namespace uartpb {

UartProtobuf::UartProtobuf(
    uart_t uart_dev, uint32_t uart_speed) :
    uart_dev_(uart_dev), uart_speed_(uart_speed)
{
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
    if (data == 0) {
        msg_t msg;
        msg.content.value = msg_length_;
        msg_send(&msg, reader_pid_);
        msg_length_ = 0;
        return;
    }
    ringbuffer_add_one(&rx_buf_, data);
    msg_length_++;
}

void UartProtobuf::message_reader()
{
    uuid_t uuid;
    msg_t msg;
    msg_t msg_queue[8];
    msg_init_queue(msg_queue, 8);

    while (1) {
        // Wait for a new message
        msg_receive(&msg);

        // Read and check message length
        uint32_t message_length = (uint32_t)msg.content.value;
        assert(message_length <= UARTPB_INPUT_MESSAGE_LENGTH_MAX);

        // Read uuid
        ringbuffer_get(&rx_buf_, (char *)&uuid, sizeof(uuid_t));
        message_length -= sizeof(uuid_t);

        // Check a handler corresponding to the uuid is registered
        if (message_handlers_.count(uuid) != 1) {
            std::cout << "Unknown message uuid: " << uuid << std::endl;
            continue;
        }

        // Read Protobuf message if any
        if (message_length > 0) {
            read_buffer_.clear();
            ringbuffer_get(&rx_buf_, (char *)read_buffer_.get_base64_data(), message_length);
            size_t res = read_buffer_.base64_decode();
            if (res == 0) {
                std::cout << "Failed to base64 decode Protobuf message (res = " << res <<  ")" << std::endl;
                continue;
            }
        }

        message_handlers_[uuid](read_buffer_);
    }
}
bool UartProtobuf::send_message(uuid_t uuid, const EmbeddedProto::MessageInterface *message)
{
    bool success = true;
    size_t base64_size = 0;
    char separator = '\n';
    mutex_lock(&mutex_);

    if (message) {
        write_buffer_.clear();
        auto serialization_status = message->serialize(write_buffer_);
        if(EmbeddedProto::Error::NO_ERRORS != serialization_status) {
            puts("Failed to serialize Protobuf message.");
            success = false;
        }
        else {
            base64_size = write_buffer_.base64_encode();
            if (base64_size == 0) {
                puts("Failed to base64 encode Protobuf serialized message.");
                success = false;
            }
        }
    }
    if (success) {
        uart_write(uart_dev_, (uint8_t *)&uuid, sizeof(uuid_t));
        if (message) {
            uart_write(uart_dev_, write_buffer_.get_base64_data(), base64_size);
        }
        uart_write(uart_dev_, (uint8_t *)&separator, 1);
    }

    mutex_unlock(&mutex_);

    return success;
}

void UartProtobuf::register_message_handler(uuid_t uuid, message_handler_t handler)
{
    message_handlers_[uuid] = handler;
}

} // namespace uartpb

} // namespace cogip
