APPLICATION = controller_robot_test

BOARD ?= cogip-native

# Motion control
USEMODULE += dualpid_meta_controller
USEMODULE += pid_ng
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
USEMODULE += shell_commands

# Misc
USEMODULE += utils

# Embedded Template Library
USEPKG += etl

INCLUDES += -I$(CURDIR)/include

include ../../Makefile.include
