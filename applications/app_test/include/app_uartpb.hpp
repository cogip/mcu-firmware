#pragma once

#include "uartpb/ReadBuffer.hpp"

namespace cogip {

namespace app {

/// Message handler for incoming Protobuf messages.
void app_uartpb_message_handler(cogip::uartpb::ReadBuffer &buffer);

} // namespace app

} // namespace cogip
