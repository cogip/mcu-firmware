# List of patch stamp files
patchstamps := $(addsuffix ed, $(realpath $(wildcard $(MCUFIRMWAREBASE)/riot-patches/*.patch)))

%.patched: %.patch
	@cd $(RIOTBASE) && quilt import $< && quilt push
	@touch $@
