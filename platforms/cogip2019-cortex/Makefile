APPLICATION = cortex

BOARD ?= cogip2019-cortex

MCUFIRMWARE_PLATFORM_BASE = cortex

# RIOT modules
USEMODULE += mpu_stack_guard
USEMODULE += scanf_float

# mcu-firmware modules
USEMODULE += cc110x
USEMODULE += pca9548
USEMODULE += sd21
USEMODULE += $(APPLICATION_MODULE)

# Board features required
FEATURES_REQUIRED += periph_i2c

USEPKG += vl53l0x-api

# Main thread stack size
CFLAGS += -DTHREAD_STACKSIZE_MAIN=4096

include ../../Makefile.include

# Override default build rule to hook a MCU footprint display
.DEFAULT_GOAL := world

.PHONY: world
world: all
	$(Q)$(MCUFIRMWAREBASE)/tools/fwsize.sh $(SIZE) $(ELFFILE) $(FLASHSIZE) $(RAMSIZE)
