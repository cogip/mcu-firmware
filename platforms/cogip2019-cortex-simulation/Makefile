APPLICATION = cortex-simulation

BOARD ?= cogip2019-cortex-native

# RIOT modules
USEMODULE += printf_float
USEMODULE += ps
USEMODULE += schedstatistics
USEMODULE += shell
USEMODULE += shell_commands
USEMODULE += xtimer

# mcu-firmware modules
USEMODULE += ctrl
USEMODULE += ctrl_quadpid
USEMODULE += motor_driver
USEMODULE += robotics
USEMODULE += $(APPLICATION_MODULE)

# Board features required
FEATURES_REQUIRED += periph_qdec
FEATURES_REQUIRED += periph_pwm

include ../../Makefile.include
