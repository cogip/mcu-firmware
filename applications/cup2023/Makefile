APPLICATION = cup2023

BOARD ?= cogip-board
#BOARD ?= cogip-native

# Platform
USEMODULE += pegasus

# Shell menus management
USEMODULE += shell_platforms
USEMODULE += shell_menu

# Main thread stack size
CFLAGS += -DTHREAD_STACKSIZE_MAIN=4096

include ../../Makefile.include
