.DEFAULT_GOAL := all

# Get absolute paths
MCUFIRMWAREBASE := $(realpath $(dir $(lastword $(MAKEFILE_LIST))))
RIOTBASE ?= $(MCUFIRMWAREBASE)/../RIOT/

# List all applications
apps := $(foreach dir,$(wildcard applications/*/Makefile),$(subst Makefile, , $(dir)))
examples := $(foreach dir,$(wildcard examples/*/Makefile),$(subst Makefile, , $(dir)))
boards := $(foreach dir,$(wildcard boards/*),$(subst boards/, , $(dir)))

# RIOT-OS patches targets
include $(MCUFIRMWAREBASE)/makefiles/riot-patches.mk.inc

.PHONY: all clean distclean doc docman doclatex docclean help world
.PHONY: distclean-riot-patches riot-patches
.PHONY: $(apps) $(examples)

all: $(apps) $(examples)	## Build all applications and examples (default target)

clean: PF_TARGET = clean
clean: $(apps) $(examples)	## Clean build for all applications and examples

distclean: PF_TARGET = distclean
distclean: $(apps) $(examples)	## Clean build and configuration for all applications and examples

$(apps) $(examples):
	"$(MAKE)" MAKEFLAGS="-j$$(nproc)" -C $@ $(PF_TARGET) || exit $$?

doc: 			## Generate doxygen
	"$(MAKE)" -BC doc/doxygen

docman:			## Generate doxygen as man pages
	"$(MAKE)" -BC doc/doxygen man

doclatex:		## Generate doxygen in latex format
	"$(MAKE)" -BC doc/doxygen latex

docclean:		## Clean documentation
	"$(MAKE)" -BC doc/doxygen clean

check-codingrules:
	$(CURDIR)/tools/check-codingrules.sh

help:			## This help message
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | sort | awk 'BEGIN {FS = ":.*?## "}; {printf "\033[36m%-30s\033[0m %s\n", $$1, $$2}'
