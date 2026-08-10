/*
 * Ethernet firmware-update service: a TFTP server that writes a pushed image
 * into the inactive riotboot slot, then reboots onto it. See fw_update_tftp.h.
 */

#include <inttypes.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "periph/pm.h"
#include "riotboot/flashwrite.h"
#include "riotboot/hdr.h"
#include "riotboot/slot.h"

#include "lwip/apps/tftp_server.h"
#include "lwip/ip4_addr.h"
#include "lwip/netif.h"
#include "lwip/pbuf.h"
#include "lwip/tcpip.h"

#include "fw_update_tftp.h"

#define ENABLE_DEBUG 0
#include "debug.h"
#define LOG_PREFIX "fw_update: "
#include "log.h"

/* lwIP names the first Ethernet netif "ET0". */
#ifndef FW_UPDATE_NETIF
#define FW_UPDATE_NETIF "ET0"
#endif

/* Static IPv4 last octet, per robot (192.168.0.<octet>). */
#ifndef FW_UPDATE_IP_LAST_OCTET
#define FW_UPDATE_IP_LAST_OCTET (100 + ROBOT_ID)
#endif

/* Offset of the start_addr field inside riotboot_hdr_t (magic, version,
 * start_addr, chksum -> 2 x uint32_t before it). */
#define HDR_START_ADDR_OFFSET (2 * sizeof(uint32_t))

static riotboot_flashwrite_t _writer;

/* Distinct, non-NULL handle returned for a slot-number query (TFTP GET). */
static char _query_handle;
static bool _query_served;

static struct
{
    bool failed;         /* transfer errored: leave slot invalid */
    size_t received;     /* total image bytes received so far      */
    uint32_t start_addr; /* start_addr parsed from the image header */
} _st;

static void* _open(const char* fname, const char* mode, u8_t write)
{
    (void)fname;
    (void)mode;

    /* A GET exposes the inactive slot number as a one-byte file, so the host
     * (make flash-net) can push the image matching that slot. */
    if (!write) {
        _query_served = false;
        return &_query_handle;
    }
    if (riotboot_slot_numof < 2) {
        LOG_ERROR(LOG_PREFIX "no second slot (not a riotboot build)\n");
        return NULL;
    }

    int target = riotboot_slot_other();
    if (riotboot_flashwrite_init(&_writer, target) != 0) {
        LOG_ERROR(LOG_PREFIX "flashwrite init failed\n");
        return NULL;
    }

    _st.failed = false;
    _st.received = 0;
    _st.start_addr = 0;
    LOG_INFO(LOG_PREFIX "receiving image into slot %d\n", target);
    return &_writer;
}

static int _write(void* handle, struct pbuf* p)
{
    riotboot_flashwrite_t* w = handle;

    if (_st.failed) {
        return -1;
    }

    for (const struct pbuf* q = p; q != NULL; q = q->next) {
        const uint8_t* data = q->payload;
        size_t len = q->len;

        /* Snoop the header start_addr (little-endian) as it streams past, to
         * reject an image built for the other slot. */
        for (size_t i = 0; i < len; i++) {
            size_t pos = _st.received + i;
            if (pos >= HDR_START_ADDR_OFFSET && pos < HDR_START_ADDR_OFFSET + sizeof(uint32_t)) {
                _st.start_addr |= (uint32_t)data[i] << (8 * (pos - HDR_START_ADDR_OFFSET));
            }
        }

        /* riotboot_flashwrite_init() holds back the "RIOT" magic (writer
         * starts at offset RIOTBOOT_FLASHWRITE_SKIPLEN), so skip the first
         * SKIPLEN bytes of the image; finish() restores them last. */
        const uint8_t* feed = data;
        size_t feed_len = len;
        if (_st.received < RIOTBOOT_FLASHWRITE_SKIPLEN) {
            size_t skip = RIOTBOOT_FLASHWRITE_SKIPLEN - _st.received;
            skip = (feed_len < skip) ? feed_len : skip;
            feed += skip;
            feed_len -= skip;
        }

        if (feed_len && riotboot_flashwrite_putbytes(w, feed, feed_len, true) != 0) {
            LOG_ERROR(LOG_PREFIX "flash write error at offset %" PRIuSIZE "\n", _st.received);
            _st.failed = true;
            return -1;
        }
        _st.received += len;
    }

    return 0;
}

static int _read(void* handle, void* buf, int bytes)
{
    /* _open() only ever returns a handle for the slot-number query on a GET
     * (PUTs go to _write), so serve that one ASCII digit once. */
    (void)handle;
    if (_query_served || bytes < 1) {
        return 0; /* EOF */
    }
    ((char*)buf)[0] = (char)('0' + riotboot_slot_other());
    _query_served = true;
    return 1;
}

static void _error(void* handle, int err, const char* msg, int size)
{
    (void)handle;
    LOG_WARNING(LOG_PREFIX "tftp error %d: %.*s\n", err, size, msg ? msg : "");
    _st.failed = true;
}

static void _close(void* handle)
{
    if (handle == &_query_handle) {
        return; /* slot-number query, nothing to finalize */
    }

    riotboot_flashwrite_t* w = handle;

    if (_st.failed) {
        LOG_WARNING(LOG_PREFIX "transfer failed, slot %d left invalid\n", w->target_slot);
        return; /* magic never written -> bootloader falls back */
    }

    /* Reject an image built for another slot: its header start_addr must fall
     * within this slot's flash region. (Cannot compare to the slot's stored
     * start_addr: the slot header is still erased at this point.) */
    uint32_t slot_base = (uint32_t)(uintptr_t)riotboot_slot_get_hdr(w->target_slot);
    uint32_t slot_end = slot_base + riotboot_slot_size(w->target_slot);
    if (_st.start_addr < slot_base || _st.start_addr >= slot_end) {
        LOG_ERROR(LOG_PREFIX "start_addr 0x%08" PRIx32 " outside slot %d "
                             "[0x%08" PRIx32 "..0x%08" PRIx32 "), rejecting\n",
                  _st.start_addr, w->target_slot, slot_base, slot_end);
        return;
    }

    if (riotboot_flashwrite_flush(w) != 0 || riotboot_flashwrite_finish(w) != 0) {
        LOG_ERROR(LOG_PREFIX "finalize failed\n");
        return;
    }

    if (riotboot_hdr_validate(riotboot_slot_get_hdr(w->target_slot)) != 0) {
        LOG_ERROR(LOG_PREFIX "written header invalid, invalidating slot\n");
        riotboot_flashwrite_invalidate(w->target_slot);
        return;
    }

    LOG_INFO(LOG_PREFIX "slot %d updated (%" PRIuSIZE " bytes), rebooting\n", w->target_slot,
             _st.received);
    pm_reboot();
}

static const struct tftp_context _tftp_ctx = {
    .open = _open,
    .close = _close,
    .read = _read,
    .write = _write,
    .error = _error,
};

void fw_update_tftp_init(void)
{
    /* lwIP is built with LWIP_TCPIP_CORE_LOCKING: every raw-API call
     * (netif_set_addr, and udp_bind inside tftp_init_server) asserts the
     * core lock is held, so do both under the lock. */
    sys_lock_tcpip_core();
    struct netif* iface = netif_find(FW_UPDATE_NETIF);
    err_t err = ERR_IF;
    if (iface != NULL) {
        ip4_addr_t ip, mask, gw;
        IP4_ADDR(&ip, 192, 168, 0, FW_UPDATE_IP_LAST_OCTET);
        IP4_ADDR(&mask, 255, 255, 255, 0);
        IP4_ADDR(&gw, 192, 168, 0, 1);
        netif_set_addr(iface, &ip, &mask, &gw);
        err = tftp_init_server(&_tftp_ctx);
    }
    sys_unlock_tcpip_core();

    if (iface == NULL) {
        LOG_ERROR(LOG_PREFIX "netif " FW_UPDATE_NETIF " not found\n");
        return;
    }

    if (err != ERR_OK) {
        LOG_ERROR(LOG_PREFIX "tftp server init failed\n");
        return;
    }

    LOG_INFO(LOG_PREFIX "tftp server ready on 192.168.0.%d\n", FW_UPDATE_IP_LAST_OCTET);
}
