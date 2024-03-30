APPLICATION = cup2023

BOARD ?= cogip-board
#BOARD ?= cogip-native

# Platform
USEMODULE += pegasus

# Main thread stack size
CFLAGS += -DTHREAD_STACKSIZE_MAIN=4096

# Update maximum number of motors for actuators
CFLAGS += -DCONFIG_MOTOR_DRIVER_MAX=3

include ../../Makefile.include
