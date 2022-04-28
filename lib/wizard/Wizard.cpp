// Project includes
#include "wizard/Wizard.hpp"
#include "uartpb_config.hpp"

namespace cogip {

namespace wizard {

static PB_OutputMessage output_message;

Wizard::Wizard(cogip::uartpb::UartProtobuf *uartpb) : uartpb_(uartpb)
{
    event_queue_init(&queue_);
    event_.super.list_node.next = nullptr;
}

void Wizard::handle_response(const PB_Message &pb_message)
{
    puts("handle_response");
    event_.pb_message = pb_message;
    event_post(&queue_, (event_t *)&event_);
}

const Wizard::PB_Message &Wizard::request(const PB_Message &request)
{
    if (! uartpb_) {
        return request;
    }
    output_message.set_wizard(request);
    uartpb_->send_message(output_message);

    wizard_event_t *event = (wizard_event_t *)event_wait(&queue_);
    return event->pb_message;
}

} // namespace wizard

} // namespace cogip

/// @}
