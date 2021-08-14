APPLICATION = lidar_obstacles

# Compatible boards: native, nucleo-f446re
BOARD ?= nucleo-f446re

USEMODULE += event
USEMODULE += printf_float
USEMODULE += ps
USEMODULE += xtimer

USEMODULE += cogip_defs
USEMODULE += lds01_dma
USEMODULE += obstacles
USEMODULE += shell_menu

INCLUDES += -I$(CURDIR)/include

CFLAGS += -DTHREAD_STACKSIZE_MAIN=4096

CFLAGS += -DCONFIG_SHELL_NO_ECHO
CFLAGS += -DCONFIG_SHELL_NO_PROMPT

include ../../Makefile.include
