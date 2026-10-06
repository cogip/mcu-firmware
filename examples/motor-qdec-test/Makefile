APPLICATION = motor-qdec-test

BOARD ?= cogip-board-h5

USEMODULE += motor_driver
USEMODULE += ztimer_msec
# enable stdio input so getchar() actually blocks on the console
USEMODULE += stdin

FEATURES_REQUIRED += periph_pwm
FEATURES_REQUIRED += periph_qdec
FEATURES_REQUIRED += periph_gpio

# motor_driver_params_t sizes its motors[] array from this
CFLAGS += -DCONFIG_MOTOR_DRIVER_MAX=2

include ../../Makefile.include
