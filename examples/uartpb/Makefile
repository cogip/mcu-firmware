APPLICATION = embedded-proto-uart

# Supported boards: cogip-native cogip-board-ng

# If no BOARD is found in the environment, use this default:
BOARD ?= cogip-board-ng

ifeq (native,$(findstring native,$(BOARD)))
	CFLAGS += "-DUART_NUMOF=2"
endif

USEMODULE += uartpb
USEMODULE += cogip_defs
USEMODULE += shell_menu
USEMODULE += ps

include ../../Makefile.include
