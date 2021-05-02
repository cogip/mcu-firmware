SRC = tracefd.c

ifeq ($(CPU),native)
	SRC += tracefd_native.c
else
	SRC += tracefd_real.c
endif

include $(RIOTBASE)/Makefile.base
