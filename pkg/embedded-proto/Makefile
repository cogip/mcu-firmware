PKG_NAME=embedded-proto
PKG_URL=https://github.com/Embedded-AMS/EmbeddedProto.git
PKG_VERSION=2.3.5
PKG_LICENSE=GPL

include $(RIOTBASE)/pkg/pkg.mk

PROTOC ?= protoc
PROTOC_GEN_EAMS ?= $(PKGDIRBASE)/embedded-proto/protoc-gen-eams
PROTOBUF_FILES ?= $(notdir $(wildcard $(APPDIR)/*.proto))
PROTO_INCLUDES += $(PROTOBUF_PATH:%=-I%) -I$(APPDIR)

all: build_ep protoc_cpp protoc_py

protoc_cpp:
	$(eval protoc_files := $(sort $(PROTOBUF_FILES) $(foreach d, $(PROTOBUF_PATH), $(notdir $(wildcard $(d)/*.proto)))))
	$(PROTOC) --plugin=protoc-gen-eams=$(PROTOC_GEN_EAMS) \
		--eams_out="$(BINDIR)/$(APPLICATION_MODULE)" $(PROTO_INCLUDES) \
		$(protoc_files)

protoc_py:
	$(eval protoc_files := $(sort $(PROTOBUF_FILES) $(foreach d, $(PROTOBUF_PATH), $(notdir $(wildcard $(d)/*.proto)))))
	$(PROTOC) --python_out="$(APPDIR)" $(PROTO_INCLUDES) \
		$(protoc_files)

build_ep:
	# Create the directory containing generated headers
	# so it can be included in all modules, even if no headers are generated yet.
	# (including not existing directories will produce compilation errors)
	mkdir -p $(BINDIR)/$(APPLICATION_MODULE)
	$(MAKE) -C $(PKG_SOURCE_DIR)/src -f $(RIOTBASE)/Makefile.base MODULE=$(PKG_NAME)
