#pragma once

// Project includes
#include "obstacles/List.hpp"
#include "uartpb/ReadBuffer.hpp"

// Obstacles of Lidar detected obstacles
cogip::obstacles::List & lidar_obstacles();

void obstacles_handler(cogip::uartpb::ReadBuffer & buffer);
