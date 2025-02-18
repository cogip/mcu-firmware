#pragma once

// Project includes
#ifdef ROBOT1
#include "robot1_conf.hpp"
#elif ROBOT2
#include "robot2_conf.hpp"
#elif ROBOT3
#include "robot3_conf.hpp"
#elif ROBOT4
#include "robot4_conf.hpp"
#elif ROBOT5
#include "robot5_conf.hpp"
#else
#error "You should build this application with the following ROBOT_ID=<1-5>. eg. make ROBOT_ID=1"
#endif
