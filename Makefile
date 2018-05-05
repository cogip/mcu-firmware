APPLICATION = cortex
RIOTBASE ?= $(CURDIR)/../RIOT
BOARD ?= cogip2018-f4xx
#BOARD ?= native
USEMODULE += xtimer
USEMODULE += motor_driver

USEMODULE += shell
USEMODULE += shell_commands
USEMODULE += schedstatistics
USEMODULE += ps
USEMODULE += printf_float
LINKFLAGS += -u _scanf_float

QUIET ?= 1

# Comment this out to disable code in RIOT that does safety checking
# which is not needed in a production environment but helps in the
# development process:
DEVELHELP ?= 1

BIN_DIRS += actuators
BIN_DIRS += calibration
BIN_DIRS += robotics
BIN_DIRS += system
BIN_USEMODULE += actuators
BIN_USEMODULE += calibration             # include "core" module
BIN_USEMODULE += robotics             # include "core" module
BIN_USEMODULE += system
BIN_USEMODULE += $(APPLICATION_MODULE)  # include application module

FEATURES_REQUIRED += periph_i2c
FEATURES_REQUIRED += periph_qdec
FEATURES_REQUIRED += periph_pwm

INCLUDES += -I$(APPDIR)/robotics/include/ -I$(APPDIR)/
ifeq ($(BOARD),native)
CFLAGS += -Wno-pedantic -Wno-unused-parameter -Wno-sign-compare
endif
CFLAGS += -DCONFIG_SD21

include $(RIOTBASE)/Makefile.include
