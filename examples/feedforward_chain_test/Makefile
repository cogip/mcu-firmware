APPLICATION = feedforward_chain_test

BOARD ?= cogip-native

# Motion control modules
USEMODULE += motion_control_common
USEMODULE += profile_feedforward_controller
USEMODULE += feedforward_combiner_controller

# Utils (required by motion_control_common)
USEMODULE += utils

# Embedded Template Library
USEPKG += etl

include ../../Makefile.include
