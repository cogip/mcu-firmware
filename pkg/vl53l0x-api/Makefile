PKG_NAME=		vl53l0x-api
PKG_URL=		https://github.com/gdoffe/vl53l0x-api.git
PKG_VERSION=	master
PKG_LICENSE=	closed

include $(RIOTBASE)/pkg/pkg.mk

.PHONY: all

CFLAGS += -Wno-unused-parameter -Wno-unused-but-set-variable

INCLUDES += -I$(PKG_BUILDDIR)/include

all:
	$(MAKE) -C $(PKG_BUILDDIR)
