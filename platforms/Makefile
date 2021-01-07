MODULE = pf

DIRS += $(MCUFIRMWARE_PLATFORM_BASE)

ifneq (,$(filter calibration,$(MCUFIRMWARE_OPTIONS)))
	DIRS += calibration
endif

include $(RIOTBASE)/Makefile.base
