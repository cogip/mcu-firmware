APPLICATION = cortex
RIOTBASE ?= $(CURDIR)/../RIOT
BOARD ?= native
USEMODULE += xtimer
USEMODULE += motor_driver

USEMODULE += shell
USEMODULE += shell_commands
USEMODULE += schedstatistics
USEMODULE += ps
USEMODULE += printf_float

QUIET ?= 1

# Comment this out to disable code in RIOT that does safety checking
# which is not needed in a production environment but helps in the
# development process:
DEVELHELP ?= 1

BIN_DIRS += calibration
BIN_DIRS += robotics
BIN_DIRS += system
BIN_USEMODULE += calibration             # include "core" module
BIN_USEMODULE += robotics             # include "core" module
BIN_USEMODULE += system
BIN_USEMODULE += $(APPLICATION_MODULE)  # include application module

FEATURES_REQUIRED += periph_qdec
FEATURES_REQUIRED += periph_pwm

INCLUDES += -I$(APPDIR)/robotics/include/ -I$(APPDIR)/

include $(RIOTBASE)/Makefile.include
