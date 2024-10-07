APPLICATION = robot-motion-control

BOARD ?= cogip-board
#BOARD ?= cogip-native

# Platform
USEMODULE += pf-robot-motion-control-ng

# Update maximum number of motors for actuators
CFLAGS += -DCONFIG_MOTOR_DRIVER_MAX=3

include ../../Makefile.include
