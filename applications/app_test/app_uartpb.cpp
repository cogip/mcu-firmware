#include "app.hpp"
#include "platform.hpp"
#include "shell_menu/shell_menu.hpp"

#include "uartpb_config.hpp"

// Read incoming Protobuf message and call the corresponding message handler
void app_message_handler(cogip::uartpb::ReadBuffer &buffer)
{
    PB_InputMessage *message = new PB_InputMessage();
    message->deserialize(buffer);

    if (message->has_command()) {
        cogip::shell::handle_pb_command(message->command());
    }
    else if (message->has_copilot_connected()) {
        pf_set_copilot_connected(true);
        puts("Copilot connected");
        if (cogip::shell::current_menu) {
            cogip::shell::current_menu->send_pb_message();
        }
    }
    else if (message->has_copilot_disconnected()) {
        pf_set_copilot_connected(false);
        puts("Copilot disconnected");
    }
    else if (message->has_wizard()) {
        pf_get_wizard()->handle_response(message->wizard());
    }
    else if (message->has_samples()) {
        app_samples_process(message->samples());
    }
    else {
        printf("Unknown response type: %" PRIu32 "\n", static_cast<uint32_t>(message->get_which_type()));
    }
    delete message;
}
