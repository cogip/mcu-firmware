#pragma once

#include "shell_menu/Menu.hpp"
#include "PB_ExampleOutputMessage.hpp"

using PB_OutputMessage = PB_ExampleOutputMessage<
    COMMAND_NAME_MAX_LENGTH,
    NB_SHELL_COMMANDS,
    COMMAND_NAME_MAX_LENGTH,
    COMMAND_DESC_MAX_LENGTH,
    64>;
