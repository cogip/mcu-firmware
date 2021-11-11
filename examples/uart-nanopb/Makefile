APPLICATION = uart-nanopb

# If no BOARD is found in the environment, use this default:
BOARD ?= cogip-nucleo-f446re
#BOARD ?= cogip-native

ifeq (native,$(findstring native,$(BOARD)))
	CFLAGS += "-DUART_NUMOF=2"
endif

FEATURES_REQUIRED += periph_uart

USEMODULE += shell_menu
USEMODULE += ps

USEPKG += nanopb

include ../../Makefile.include
