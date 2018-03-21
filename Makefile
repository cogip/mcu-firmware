APPLICATION = cortex
RIOTBASE ?= $(CURDIR)/../RIOT
BOARD ?= native
USEMODULE += xtimer
USEMODULE += motor_driver

USEMODULE += shell
USEMODULE += shell_commands
USEMODULE += schedstatistics
USEMODULE += ps

QUIET ?= 1

CFLAGS += -DCONFIG_CALIBRATION
# Comment this out to disable code in RIOT that does safety checking
# which is not needed in a production environment but helps in the
# development process:
DEVELHELP ?= 1

BIN_DIRS += core
BIN_DIRS += system
BIN_USEMODULE += core                   # include "core" module
BIN_USEMODULE += system
BIN_USEMODULE += $(APPLICATION_MODULE)  # include application module

INCLUDES += -I$(APPDIR)/core/include/ -I$(APPDIR)/

include $(RIOTBASE)/Makefile.include
