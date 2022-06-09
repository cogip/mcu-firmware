#include "app.hpp"
#include "platform.hpp"
#include "shell_menu/shell_menu.hpp"

#include "uartpb_config.hpp"

#include <iostream>

namespace cogip {

namespace app {

// Read incoming Protobuf message and call the corresponding message handler
void app_uartpb_message_handler(cogip::uartpb::ReadBuffer &buffer)
{
    PB_InputMessage *message = new PB_InputMessage();
    EmbeddedProto::Error error = message->deserialize(buffer);
    if (error != EmbeddedProto::Error::NO_ERRORS) {
        COGIP_DEBUG_COUT("Protobuf deserialization error: " << static_cast<int>(error));
        return;
    }
    if (message->has_command()) {
        cogip::shell::handle_pb_command(message->command());
    }
    else if (message->has_copilot_connected()) {
        pf_set_copilot_connected(true);
        COGIP_DEBUG_COUT("Copilot connected");
        if (cogip::shell::current_menu) {
            cogip::shell::current_menu->send_pb_message();
        }
    }
    else if (message->has_copilot_disconnected()) {
        pf_set_copilot_connected(false);
        COGIP_DEBUG_COUT("Copilot disconnected");
    }
    else if (message->has_wizard()) {
        pf_get_wizard()->handle_response(message->wizard());
    }
    else if (message->has_samples()) {
        app_samples_process(message->samples());
    }
    else {
        COGIP_DEBUG_CERR("Unknown response type: " << static_cast<uint32_t>(message->get_which_type()));
    }
    delete message;
}

} // namespace app

} // namespace cogip
