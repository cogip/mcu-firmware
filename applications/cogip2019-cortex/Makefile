APPLICATION = cortex

BOARD ?= cogip2019-cortex

MCUFIRMWARE_PLATFORM_BASE = cortex

# Main thread stack size
CFLAGS += -DTHREAD_STACKSIZE_MAIN=4096
# Number of positions available for each servomotor
CFLAGS += -DSD21_SERVO_POS_NUMOF=3

include ../../Makefile.include
