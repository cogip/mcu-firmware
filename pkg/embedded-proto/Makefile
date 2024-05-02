PKG_NAME=embedded-proto
PKG_URL=https://github.com/Embedded-AMS/EmbeddedProto.git
PKG_VERSION=3.4.2
PKG_LICENSE=GPL

include $(RIOTBASE)/pkg/pkg.mk

all: embedded_proto_options $(BINDIR)/protoc_cpp $(BINDIR)/protoc_py

PROTOC ?= protoc
PROTOC_GEN_EAMS ?= $(PKGDIRBASE)/embedded-proto/protoc-gen-eams
PROTOBUF_FILES ?= $(wildcard $(APPDIR)/*.proto)
PROTOC_FILES = $(sort $(PROTOBUF_FILES) $(foreach d, $(PROTOBUF_PATH), $(wildcard $(d)/*.proto)))
PROTO_INCLUDES += $(PROTOBUF_PATH:%=-I%) -I$(APPDIR)

embedded_proto_options:
	@cd $(PKG_SOURCE_DIR)/generator; protoc -I . --python_out=EmbeddedProto embedded_proto_options.proto

%/protoc_cpp: $(PROTOC_FILES) $(BINDIR)/$(PKG_NAME)
	$(PROTOC) --plugin=protoc-gen-eams=$(PROTOC_GEN_EAMS) \
		--eams_out="$(BINDIR)/$(APPLICATION_MODULE)" $(PROTO_INCLUDES) \
		$(PROTOC_FILES)
	@touch $@

%/protoc_py: $(PROTOC_FILES) $(BINDIR)/$(PKG_NAME)
	$(PROTOC) --python_out="$(APPDIR)" $(PROTO_INCLUDES) \
		$(PROTOC_FILES)
	@touch $@

%/$(PKG_NAME):
	# Create the directory containing generated headers
	# so it can be included in all modules, even if no headers are generated yet.
	# (including not existing directories will produce compilation errors)
	mkdir -p $(BINDIR)/$(APPLICATION_MODULE)
	$(MAKE) -C $(PKG_SOURCE_DIR)/src -f $(RIOTBASE)/Makefile.base MODULE=$(PKG_NAME)
