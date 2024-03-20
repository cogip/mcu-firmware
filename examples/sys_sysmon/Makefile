APPLICATION = sys_sysmon_example

BOARD ?= cogip-board

ifeq (native,$(findstring native,$(BOARD)))
	CFLAGS += "-DUART_NUMOF=2"
endif

USEMODULE += shell_cmd_heap
USEMODULE += ps
USEMODULE += shell
USEMODULE += shell_cmds_default
USEMODULE += sysmon
USEMODULE += uartpb

include ../../Makefile.include
