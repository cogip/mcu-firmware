APPLICATION = cortex

BOARD ?= cogip-board

# Platform
USEMODULE += cortex

# Shell menus management
USEMODULE += shell_menu
USEMODULE += shell_platforms
USEMODULE += shell_planners
USEMODULE += shell_quadpid
USEMODULE += shell_sd21
USEMODULE += shell_pca9548

# Main thread stack size
CFLAGS += -DTHREAD_STACKSIZE_MAIN=4096
# Number of positions available for each servomotor
CFLAGS += -DSD21_SERVO_POS_NUMOF=3

include ../../Makefile.include
