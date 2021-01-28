# List all applications
apps := $(foreach dir,$(wildcard applications/*/Makefile),$(subst Makefile, , $(dir)))
boards := $(foreach dir,$(wildcard boards/*),$(subst boards/, , $(dir)))

.PHONY: all clean distclean doc docman doclatex docclean help $(apps)

all: $(apps)		## Build all applications (default target)

clean: PF_TARGET = clean
clean: $(apps)		## Clean build for all applications

distclean: PF_TARGET = distclean
distclean: $(apps)	## Clean build and configuration for all applications

$(apps):
	$(MAKE) -j$$(nproc) -C $@ $(PF_TARGET) || exit $$?; \

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
