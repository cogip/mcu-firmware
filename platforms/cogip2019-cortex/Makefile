APPLICATION = cortex

BOARD ?= cogip2019-cortex

MCUFIRMWARE_PLATFORM_BASE = cogip2019-cortex-common

# RIOT modules
USEMODULE += printf_float
USEMODULE += ps
USEMODULE += schedstatistics
USEMODULE += shell
USEMODULE += shell_commands
USEMODULE += xtimer

# mcu-firmware modules
USEMODULE += ctrl
USEMODULE += quadpid
USEMODULE += motor_driver
USEMODULE += pca9548
USEMODULE += robotics
USEMODULE += sd21
USEMODULE += $(APPLICATION_MODULE)

# Options
MCUFIRMWARE_OPTIONS += calibration

# Board features required
FEATURES_REQUIRED += periph_i2c
FEATURES_REQUIRED += periph_qdec
FEATURES_REQUIRED += periph_pwm

USEPKG += vl53l0x-api

include ../../Makefile.include
