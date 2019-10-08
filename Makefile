# List all platforms
pfs := $(foreach dir,$(wildcard platforms/*/Makefile),$(subst Makefile, , $(dir)))

.PHONY: all clean distclean doc doc-man latex docclean help $(pfs)

all: $(pfs)

clean: PF_TARGET = clean
clean: $(pfs)

distclean: PF_TARGET = distclean
distclean: $(pfs)

$(pfs):
	$(MAKE) -j$$(nproc) -C $@ $(PF_TARGET)


doc:
	"$(MAKE)" -BC doc/doxygen

doc-man:
	"$(MAKE)" -BC doc/doxygen man

latex:
	"$(MAKE)" -BC doc/doxygen latex

docclean:
	"$(MAKE)" -BC doc/doxygen clean

