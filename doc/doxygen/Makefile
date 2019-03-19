MCUFIRMWAREBASE=$(shell git rev-parse --show-toplevel)
# Generate list of quoted absolute include paths. Evaluated in cogip.doxyfile.
export STRIP_FROM_INC_PATH_LIST=$(shell \
    git ls-tree -dr --full-tree --name-only HEAD controllers drivers |\
    grep '/include$$' |\
    sed 's/.*/\"$(subst /,\/,$(MCUFIRMWAREBASE))\/\0\"/')

# use lessc (http://lesscss.org/#using-less) for compiling CSS
# It can also be installed in ubuntu with the `node-less` package
LESSC ?= $(shell command -v lessc 2>/dev/null)

.PHONY: doc
doc: html

# by marking html as phony we force make to re-run Doxygen even if the directory exists.
.PHONY: html
html: src/css/cogip.css
	( cat cogip.doxyfile ; echo "GENERATE_HTML = yes" ) | doxygen -

.PHONY: man
man: src/changelog.md
	( cat cogip.doxyfile ; echo "GENERATE_MAN = yes" ) | doxygen -

ifneq (,$(LESSC))
src/css/cogip.css: src/css/cogip.less src/css/variables.less
	@$(LESSC) $< $@

src/css/variables.less: src/config.json
	@grep "^\s*\"@" $< | sed -e 's/^\s*"//g' -e 's/":\s*"/: /g' \
	  -e 's/",\?$$/;/g' -e 's/\\"/"/g' > $@
endif

.PHONY:
latex:
	( cat cogip.doxyfile ; echo "GENERATE_LATEX= yes" ) | doxygen -

clean:
	-@rm -rf latex man html doxygen_objdb_*.tmp doxygen_entrydb_*.tmp src/changelog.md
