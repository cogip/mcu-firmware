APPLICATION = cortex

BOARD ?= cogip2019-cortex

MCUFIRMWARE_PLATFORM_BASE = cogip2019-cortex-common

# RIOT modules
USEMODULE += mpu_stack_guard
USEMODULE += motor_driver
USEMODULE += printf_float
USEMODULE += scanf_float
USEMODULE += ps
USEMODULE += schedstatistics
USEMODULE += shell
USEMODULE += shell_commands
USEMODULE += xtimer

# mcu-firmware modules
USEMODULE += ctrl
USEMODULE += planner
USEMODULE += pca9548
USEMODULE += quadpid
USEMODULE += robotics
USEMODULE += cc110x
USEMODULE += sd21
USEMODULE += $(APPLICATION_MODULE)

# Options
#MCUFIRMWARE_OPTIONS += calibration

# Board features required
FEATURES_REQUIRED += periph_i2c
FEATURES_REQUIRED += periph_qdec
FEATURES_REQUIRED += periph_pwm

USEPKG += vl53l0x-api

# Main thread stack size
CFLAGS += -DTHREAD_STACKSIZE_MAIN=4096

DEVELHELP=1

include ../../Makefile.include

# Override default build rule to hook a MCU footprint display
.DEFAULT_GOAL := world

.PHONY: world
world: all
	$(Q)$(MCUFIRMWAREBASE)/tools/fwsize.sh $(SIZE) $(ELFFILE) $(FLASHSIZE) $(RAMSIZE)
