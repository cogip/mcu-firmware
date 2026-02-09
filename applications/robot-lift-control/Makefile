APPLICATION = robot-lift-control

BOARD ?= cogip-board

# Lift identifier (1-5)
LIFT_ID ?= 1
CFLAGS += -DLIFT_ID=$(LIFT_ID)

# Platform
USEMODULE += pf-robot-motors

# Update maximum number of motors for actuators
CFLAGS += -DCONFIG_MOTOR_DRIVER_MAX=2

include ../../Makefile.include
