// Project includes
#include "wizard/Wizard.hpp"

#include <iostream>

namespace cogip {

namespace wizard {

constexpr cogip::uartpb::uuid_t wizard_uuid = 1525532810;

Wizard::Wizard(cogip::uartpb::UartProtobuf & uartpb) : uartpb_(uartpb)
{
    event_queue_init_detached(&queue_);
    event_.super.list_node.next = nullptr;
    uartpb_.register_message_handler(wizard_uuid, std::bind(&Wizard::handle_response, this, std::placeholders::_1));
}

void Wizard::handle_response(cogip::uartpb::ReadBuffer & buffer)
{
    EmbeddedProto::Error error = event_.pb_message.deserialize(buffer);
    if (error != EmbeddedProto::Error::NO_ERRORS) {
        std::cout << "Wizard: Protobuf deserialization error: " << static_cast<int>(error) << std::endl;
        return;
    }
    event_post(&queue_, (event_t *)&event_);
}

const Wizard::PB_Message &Wizard::request(const PB_Message &request)
{
    if (! queue_claimed_) {
        event_queue_claim(&queue_);
        queue_claimed_ = true;
    }

    uartpb_.send_message(wizard_uuid, &request);

    wizard_event_t *event = (wizard_event_t *)event_wait(&queue_);
    return event->pb_message;
}

} // namespace wizard

} // namespace cogip

/// @}
