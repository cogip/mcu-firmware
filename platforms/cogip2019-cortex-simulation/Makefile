APPLICATION = cortex-simulation

BOARD ?= cogip2019-cortex-native

MCUFIRMWARE_PLATFORM_BASE = cogip2019-cortex-common

# RIOT modules
USEMODULE += motor_driver
USEMODULE += printf_float
USEMODULE += ps
USEMODULE += schedstatistics
USEMODULE += shell
USEMODULE += shell_commands
USEMODULE += xtimer

# mcu-firmware modules
USEMODULE += ctrl
USEMODULE += planner
USEMODULE += quadpid
USEMODULE += robotics
USEMODULE += $(APPLICATION_MODULE)

# Options
MCUFIRMWARE_OPTIONS += calibration

# Board features required
FEATURES_REQUIRED += periph_qdec
FEATURES_REQUIRED += periph_pwm

include ../../Makefile.include
