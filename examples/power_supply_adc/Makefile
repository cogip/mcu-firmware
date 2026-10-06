APPLICATION = power_supply_adc_example

BOARD ?= cogip-board

# The power supply board embeds a MAX11615 (I2C address 0x33)
MAX1161X_VARIANT ?= max11615

USEMODULE += $(MAX1161X_VARIANT)

USEMODULE += saul_reg
USEMODULE += shell
USEMODULE += shell_cmd_saul_reg
USEMODULE += shell_cmd_i2c_scan
USEMODULE += ztimer_msec

include ../../Makefile.include
