#pragma once

#include "shell_menu/Menu.hpp"
#include "obstacles/List.hpp"
#include "obstacles/Obstacle.hpp"
#include "avoidance.hpp"

#include "PB_GameInputMessage.hpp"
#include "PB_GameOutputMessage.hpp"

using PB_InputMessage = PB_GameInputMessage<
    COMMAND_NAME_MAX_LENGTH,
    COMMAND_DESC_MAX_LENGTH>;

using PB_OutputMessage = PB_GameOutputMessage<
    COMMAND_NAME_MAX_LENGTH,
    NB_SHELL_COMMANDS,
    COMMAND_NAME_MAX_LENGTH,
    COMMAND_DESC_MAX_LENGTH,
    AVOIDANCE_GRAPH_MAX_VERTICES,
    OBSTACLES_MAX_NUMBER,
    OBSTACLE_BOUNDING_BOX_VERTICES>;
