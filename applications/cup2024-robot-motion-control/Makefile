APPLICATION = cup2024-robot-motion-control

BOARD ?= cogip-board
#BOARD ?= cogip-native

# Platform
USEMODULE += pf-robot-motion-control

# Main thread stack size
CFLAGS += -DTHREAD_STACKSIZE_MAIN=8192

# Update maximum number of motors for actuators
CFLAGS += -DCONFIG_MOTOR_DRIVER_MAX=3

include ../../Makefile.include
