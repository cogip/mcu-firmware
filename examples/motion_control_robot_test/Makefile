APPLICATION = controller_robot_test

BOARD ?= cogip-native

# Motion control
USEMODULE += dualpid_meta_controller
USEMODULE += encoder
USEMODULE += motor
USEMODULE += motor_driver
USEMODULE += localization
USEMODULE += pid
USEMODULE += platform_engine
USEMODULE += polar_parallel_meta_controller
USEMODULE += pose_pid_controller
USEMODULE += pose_straight_filter
USEMODULE += quadpid_meta_controller
USEMODULE += speed_pid_controller
USEMODULE += speed_filter

# Shell
USEMODULE += ps
USEMODULE += shell
USEMODULE += shell_cmds_default

# Misc
USEMODULE += utils

# Embedded Template Library
USEPKG += etl

INCLUDES += -I$(CURDIR)/include

include ../../Makefile.include
