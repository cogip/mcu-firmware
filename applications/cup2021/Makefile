APPLICATION = cup2021

BOARD ?= cogip-board

# Platform
USEMODULE += cortex

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
CFLAGS += -DDEBUG_ASSERT_VERBOSE

include ../../Makefile.include
