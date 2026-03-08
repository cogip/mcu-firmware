#include "canpb/CanProtobuf.hpp"

// System includes
#include "can/can.h"
#include "log.h"
#include <inttypes.h>

// RIOT includes
#include "Errors.h"

#define ENABLE_DEBUG 0
#include <debug.h>

#define MESSAGE_READER_THREAD_MSG_QUEUE_SIZE 8

// CAN message types
#define CAN_MSG_RECV 0x400

namespace cogip {

namespace canpb {

CanProtobuf::CanProtobuf(uint8_t can_interface_number) : can_interface_number_(can_interface_number)
{
}

bool CanProtobuf::init(struct can_filter* filter)
{
    LOG_INFO("Initialize CanProtobuf %" PRIu8 "\n", can_interface_number_);
    return conn_can_raw_create(&conn_can_raw_, filter, 1, can_interface_number_, 0);
}

void CanProtobuf::start_reader()
{
    reader_pid_ = thread_create(reader_stack_, sizeof(reader_stack_), CANPB_READER_PRIO,
                                THREAD_CREATE_STACKTEST, message_reader_wrapper,
                                static_cast<void*>(this), "Protobuf reader");
}

void CanProtobuf::message_reader()
{
    can_frame_t frame;

    LOG_INFO("Waiting for messages...\n");

    while (conn_can_raw_recv(&conn_can_raw_, &frame, 0) == sizeof(can_frame_t)) {

        uuid_t uuid = frame.can_id & CAN_EFF_MASK;

        // Check a handler corresponding to the uuid is registered
        if (message_handlers_.count(uuid) != 1) {
            DEBUG("Unknown message uuid: 0x%" PRIx32 "\n", static_cast<uint32_t>(uuid));
            continue;
        }
        DEBUG("receive message uuid: 0x%" PRIx32 "\n", static_cast<uint32_t>(uuid));

        // Read Protobuf message if any
        if (frame.len > 0) {
            read_buffer_.clear();
            memcpy(read_buffer_.get_base64_data(), frame.data, frame.len);
            size_t res = read_buffer_.base64_decode();
            if (res == 0) {
                LOG_ERROR("Failed to base64 decode Protobuf message (res = %zu)\n", res);
                continue;
            }
        }

        message_handlers_[uuid](read_buffer_);
    }

    LOG_INFO("Stop waiting for messages...\n");
}

bool CanProtobuf::send_message(uuid_t uuid, const EmbeddedProto::MessageInterface* message)
{
    bool success = true;
    size_t base64_size = 0;
    mutex_lock(&mutex_);

    if (message) {
        write_buffer_.clear();
        auto serialization_status = message->serialize(write_buffer_);
        if (EmbeddedProto::Error::NO_ERRORS != serialization_status) {
            LOG_ERROR("Failed to serialize Protobuf message\n");
            success = false;
        } else if (write_buffer_.get_size() > 0) {
            base64_size = write_buffer_.base64_encode();
            if (base64_size == 0) {
                LOG_ERROR("Failed to base64 encode Protobuf serialized message\n");
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
        } else {
            LOG_ERROR("Bad message length(%zu) for uuid: 0x%" PRIx32 "\n", base64_size,
                      static_cast<uint32_t>(uuid));
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
