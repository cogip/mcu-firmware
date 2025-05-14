#include "canpb/CanProtobuf.hpp"

// System includes
#include <iostream>
#include "can/can.h"

// RIOT includes
#include "Errors.h"

#define MESSAGE_READER_THREAD_MSG_QUEUE_SIZE    8

// CAN message types
#define CAN_MSG_RECV        0x400

namespace cogip {

namespace canpb {

CanProtobuf::CanProtobuf(
    uint8_t can_interface_number) :
    can_interface_number_(can_interface_number)
{
}

bool CanProtobuf::init(struct can_filter *filter)
{
    std::cout << "Initialiaze CanProtobuf " << can_interface_number_ << std::endl;
    return conn_can_raw_create(&conn_can_raw_, filter, 1, can_interface_number_, 0);
}

void CanProtobuf::start_reader()
{
    reader_pid_ = thread_create(
        reader_stack_,
        sizeof(reader_stack_),
        CANPB_READER_PRIO,
        THREAD_CREATE_STACKTEST,
        message_reader_wrapper,
        (void *)this,
        "Protobuf reader"
        );
}

void CanProtobuf::message_reader()
{
    can_frame_t frame;

    std::cout << "Waiting for messages..." << std::endl;

    while (conn_can_raw_recv(&conn_can_raw_, &frame,
           0) == sizeof(can_frame_t)) {

        uuid_t uuid = frame.can_id & CAN_EFF_MASK;

        // Check a handler corresponding to the uuid is registered
        if (message_handlers_.count(uuid) != 1) {
            //std::cout << "Unknown message uuid: " << (uint32_t)uuid << std::endl;
            continue;
        }
        std::cout << "receive message uuid: 0x" << std::hex << (uint32_t)uuid << std::dec << std::endl;

        // Read Protobuf message if any
        if (frame.len > 0) {
            read_buffer_.clear();
            memcpy(read_buffer_.get_base64_data(), frame.data, frame.len);
            size_t res = read_buffer_.base64_decode();
            if (res == 0) {
                std::cout << "Failed to base64 decode Protobuf message (res = " << res <<  ")" << std::endl;
                continue;
            }
        }

        message_handlers_[uuid](read_buffer_);
    }

    std::cout << "Stop waiting for messages..." << std::endl;
}

bool CanProtobuf::send_message(uuid_t uuid, const EmbeddedProto::MessageInterface *message)
{
    bool success = true;
    size_t base64_size = 0;
    mutex_lock(&mutex_);

    if (message) {
        write_buffer_.clear();
        auto serialization_status = message->serialize(write_buffer_);
        if(EmbeddedProto::Error::NO_ERRORS != serialization_status) {
            puts("Failed to serialize Protobuf message.");
            success = false;
        }
        else if (write_buffer_.get_size() > 0) {
            base64_size = write_buffer_.base64_encode();
            if (base64_size == 0) {
                puts("Failed to base64 encode Protobuf serialized message.");
                success = false;
            }
        }
    }
    if (success) {
        if (base64_size < DEFAULT_CAN_MAX_DLEN) {
            can_frame_t frame;
            frame.can_id = uuid | CAN_EFF_FLAG;
            frame.len = base64_size;
            frame.flags = CANFD_FDF;
            for (uint8_t byte_index = 0; byte_index < base64_size; byte_index++) {
                frame.data[byte_index] = write_buffer_.get_base64_data()[byte_index];
            }
            conn_can_raw_t conn;
            conn_can_raw_create(&conn, NULL, 0, can_interface_number_, 0);
            conn_can_raw_send(&conn, &frame, 0);
            conn_can_raw_close(&conn);
        }
        else {
            std::cerr << "Bad message length(" << base64_size << ") for uuid: " << uuid << std::endl;
        }
    }

    mutex_unlock(&mutex_);

    return success;
}

void CanProtobuf::register_message_handler(uuid_t uuid, message_handler_t handler)
{
    message_handlers_[uuid] = handler;
}

} // namespace canpb

} // namespace cogip
