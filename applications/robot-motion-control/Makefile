APPLICATION = robot-motion-control

BOARD ?= cogip-board
#BOARD ?= cogip-native

ROBOT_ID?=1

# Platform
USEMODULE += pf-robot-motion-control

# Update maximum number of motors for actuators
CFLAGS += -DCONFIG_MOTOR_DRIVER_MAX=2
CFLAGS += -DROBOT$(ROBOT_ID)

include ../../Makefile.include
