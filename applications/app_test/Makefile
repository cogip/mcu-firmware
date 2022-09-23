APPLICATION = app_test

BOARD ?= cogip-nucleo-f446re
#BOARD ?= cogip-native

# Platform
USEMODULE += pf_test

# Shell menus management
USEMODULE += shell_platforms
USEMODULE += shell_menu
USEMODULE += shell_planners
USEMODULE += shell_quadpid

ifeq ($(CPU),native)
	USEMODULE += shmem
endif

# Main thread stack size
CFLAGS += -DTHREAD_STACKSIZE_MAIN=4096

# Customize containers
CFLAGS += -DPATH_MAX_POSES=6

include ../../Makefile.include
