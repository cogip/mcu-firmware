APPLICATION = robot-motion-control

BOARD ?= cogip-board
#BOARD ?= cogip-native

# Display a warning message in case of default value
ifeq ($(ROBOT_ID),)
..build-message: ..build-message-robot-id-warning
else
..build-message: ..build-message-robot-id
endif
ROBOT_ID ?= 1

# Platform
USEMODULE += pf-robot-motion-control

# Update maximum number of motors for actuators
CFLAGS += -DCONFIG_MOTOR_DRIVER_MAX=2
CFLAGS += -DROBOT_ID=$(ROBOT_ID)

include ../../Makefile.include

..build-message-robot-id:
	@$(COLOR_ECHO) '$(COLOR_GREEN)Using ROBOT_ID = "$(ROBOT_ID)".$(COLOR_RESET)'
	@$(COLOR_ECHO)

..build-message-robot-id-warning:
	@$(COLOR_ECHO) '$(COLOR_YELLOW)Using default ROBOT_ID = "$(ROBOT_ID)".$(COLOR_RESET)'
	@$(COLOR_ECHO)
