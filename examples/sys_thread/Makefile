APPLICATION = sys_sysmon_example

BOARD ?= cogip-nucleo-f446re

ifeq (native,$(findstring native,$(BOARD)))
	CFLAGS += "-DUART_NUMOF=2"
endif

USEMODULE += heap_cmd
USEMODULE += ps
USEMODULE += shell
USEMODULE += shell_commands
USEMODULE += sysmon
USEMODULE += uartpb

include ../../Makefile.include
