APPLICATION = cortex

BOARD ?= cogip2019-cortex

# Platform
USEMODULE += pf_cortex

# Planner
USEMODULE += pl_astar

# Main thread stack size
CFLAGS += -DTHREAD_STACKSIZE_MAIN=4096
# Number of positions available for each servomotor
CFLAGS += -DSD21_SERVO_POS_NUMOF=3

include ../../Makefile.include
