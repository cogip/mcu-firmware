APPLICATION = sdcard

BOARD ?= cogip-nucleo-f446re

USEMODULE += shell
USEMODULE += shell_commands
USEMODULE += ps
USEMODULE += mtd_sdcard
USEMODULE += vfs
USEMODULE += fatfs_vfs

INCLUDES += -I$(CURDIR)/include

CFLAGS += -DSD_CARD_SPI_MODE=SPI_MODE_3

include ../../Makefile.include
