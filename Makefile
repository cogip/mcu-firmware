APPLICATION = cortex
RIOTBASE ?= $(CURDIR)/../RIOT
RIOTBOARD ?= boards/
BOARD ?= cogip2019-cortex
USEMODULE += xtimer
USEMODULE += motor_driver

USEMODULE += shell
USEMODULE += shell_commands
USEMODULE += schedstatistics
USEMODULE += ps
USEMODULE += printf_float
LINKFLAGS += -u _scanf_float

USEMODULE += quadpid
DIRS += controllers

QUIET ?= 1

# Comment this out to disable code in RIOT that does safety checking
# which is not needed in a production environment but helps in the
# development process:
DEVELHELP ?= 1

DIRS += robotics
USEMODULE += robotics             # include "core" module
USEMODULE += $(APPLICATION_MODULE)  # include application module

#FEATURES_REQUIRED += periph_i2c
FEATURES_REQUIRED += periph_qdec
FEATURES_REQUIRED += periph_pwm

INCLUDES += -I$(APPDIR)/robotics/include/ -I$(APPDIR)/
INCLUDES += -I$(APPDIR)/controllers/include/

CFLAGS += -Wno-pedantic -Wno-unused-parameter -Wno-sign-compare
CFLAGS += -DCONFIG_USE_STARTER

include $(RIOTBASE)/Makefile.include
