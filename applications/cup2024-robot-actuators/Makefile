APPLICATION = cup2024-robot-actuators

BOARD ?= cogip-board
#BOARD ?= cogip-native

# Platform
USEMODULE += pf-robot-actuators

# Main thread stack size
CFLAGS += -DTHREAD_STACKSIZE_MAIN=4096

# Update maximum number of motors for actuators
CFLAGS += -DCONFIG_MOTOR_DRIVER_MAX=2

include ../../Makefile.include
