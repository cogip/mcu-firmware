# Network firmware update: build a fresh slot image and push it into the
# robot's inactive riotboot slot over TFTP (see lib/fw_update_tftp).
#
# The device serves its inactive slot number as the virtual file "slot", so
# flash-net always pushes the image matching that slot (no guessing, no
# wrong-slot writes). On success the robot reboots into the new image.
#
# Target IP defaults to the per-robot address the updater assigns
# (192.168.0.100+ROBOT_ID); override with NET_TARGET_IP=<ip>.
NET_TARGET_IP ?= 192.168.0.$(shell expr 100 + $(ROBOT_ID))

.PHONY: flash-net
flash-net: riotboot
	@command -v curl >/dev/null 2>&1 || { echo "flash-net: curl is required"; exit 1; }
	@echo "flash-net: querying inactive slot on $(NET_TARGET_IP)"
	@slot=$$(curl -s --max-time 5 tftp://$(NET_TARGET_IP)/slot); \
	case "$$slot" in \
	  0) bin="$(SLOT0_RIOT_BIN)";; \
	  1) bin="$(SLOT1_RIOT_BIN)";; \
	  *) echo "flash-net: unexpected slot reply '$$slot' (is $(NET_TARGET_IP) reachable?)"; exit 1;; \
	esac; \
	echo "flash-net: pushing $$(basename $$bin) into slot $$slot"; \
	curl -s --max-time 120 -T $$bin tftp://$(NET_TARGET_IP)/fw.bin; \
	echo "flash-net: image sent, robot rebooting into slot $$slot"
