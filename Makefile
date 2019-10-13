# List all platforms
pfs := $(foreach dir,$(wildcard platforms/*/Makefile),$(subst Makefile, , $(dir)))

.PHONY: all clean distclean doc docman doclatex docclean help $(pfs)

all: $(pfs)		## Build all platforms (default target)

clean: PF_TARGET = clean
clean: $(pfs)		## Clean build for all platforms

distclean: PF_TARGET = distclean
distclean: $(pfs)	## Clean build and configuration for all platforms

$(pfs):
	$(MAKE) -j$$(nproc) -C $@ $(PF_TARGET)


doc: 			## Generate doxygen
	"$(MAKE)" -BC doc/doxygen

docman:			## Generate doxygen as man pages
	"$(MAKE)" -BC doc/doxygen man

doclatex:		## Generate doxygen in latex format
	"$(MAKE)" -BC doc/doxygen latex

docclean:		## Clean documentation
	"$(MAKE)" -BC doc/doxygen clean

help:			## This help message
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | sort | awk 'BEGIN {FS = ":.*?## "}; {printf "\033[36m%-30s\033[0m %s\n", $$1, $$2}'
