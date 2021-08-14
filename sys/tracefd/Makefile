SRCXX = tracefd.cpp

ifeq ($(CPU),native)
	SRCXX += tracefd_native.cpp
else
	SRCXX += tracefd_real.cpp
endif

include $(RIOTBASE)/Makefile.base
