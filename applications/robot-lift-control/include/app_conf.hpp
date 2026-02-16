#pragma once

// Macro helpers for LIFT_ID → liftX_conf.hpp include
#define STRINGIFY(x) #x
#define EXPAND_AND_STRINGIFY(x) STRINGIFY(x)
#define CONCATENATE3(a, b, c) a##b##c
#define LIFT_CONF_FILE(id) CONCATENATE3(lift, id, _conf.hpp)

#ifndef LIFT_ID
#error "Build with LIFT_ID=<1-2>. Ex: make LIFT_ID=1"
#endif

#if LIFT_ID < 1 || LIFT_ID > 2
#error "LIFT_ID must be 1 or 2"
#endif

// Include the appropriate lift configuration based on LIFT_ID
#include EXPAND_AND_STRINGIFY(LIFT_CONF_FILE(LIFT_ID))
