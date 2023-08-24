# List of patch stamp files
patchstamps := $(addsuffix ed, $(realpath $(wildcard $(MCUFIRMWAREBASE)/riot-patches/*.patch)))

# RIOT-OS patches targets must not be built in parallel
ifneq ($(filter %riot-patches,$(MAKECMDGOALS)),)
.NOTPARALLEL: distclean-riot-patches riot-patches
endif

%.patched: %.patch
	@cd $(RIOTBASE) && git am --keep-cr --signoff < $<
	@touch $@

riot-patches: $(patchstamps)

distclean-riot-patches:
	$(eval _patchstamps_number=`ls $(MCUFIRMWAREBASE)/riot-patches/*.patched -1 2>/dev/null | wc -l`)
	@cd $(RIOTBASE) && git reset --hard `git branch --show-current `~$(_patchstamps_number) && git clean -dxf
	@rm -f $(patchstamps)
