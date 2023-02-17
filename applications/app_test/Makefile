APPLICATION = app_test

BOARD ?= cogip-nucleo-f446re
#BOARD ?= cogip-native

# Platform
USEMODULE += pf_test

# Shell menus management
USEMODULE += shell_platforms
USEMODULE += shell_menu

# Main thread stack size
CFLAGS += -DTHREAD_STACKSIZE_MAIN=4096

include ../../Makefile.include
