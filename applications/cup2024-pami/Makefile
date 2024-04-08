APPLICATION = cup2024-pami

BOARD ?= cogip-board-pami
#BOARD ?= cogip-native

# Platform
USEMODULE += pf-pami

# Main thread stack size
CFLAGS += -DTHREAD_STACKSIZE_MAIN=4096

# Update maximum number of motors for actuators
CFLAGS += -DCONFIG_MOTOR_DRIVER_MAX=3

include ../../Makefile.include
