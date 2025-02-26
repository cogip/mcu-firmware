#pragma once

// Macro to generate the configuration file name
#define STRINGIFY(x) #x
#define EXPAND_AND_STRINGIFY(x) STRINGIFY(x)
#define CONCATENATE(prefix, id, suffix) prefix##id##suffix
#define ROBOT_CONF_FILE(id) CONCATENATE(robot, id, _conf.hpp)

// Should be set on build command line
#ifndef ROBOT_ID
#error "You should build this application with ROBOT_ID=<1-5>. e.g., make ROBOT_ID=1"
#endif

#include EXPAND_AND_STRINGIFY(ROBOT_CONF_FILE(ROBOT_ID))
