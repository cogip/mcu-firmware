/*
 * Confirmed-boot guard with automatic riotboot rollback. See boot_guard.h.
 */

#include <stdbool.h>
#include <stdint.h>

#include "irq.h"
#include "periph/flashpage.h"
#include "periph/pm.h"
#include "periph/wdt.h"
#include "riotboot/slot.h"
#include "thread.h"
#include "ztimer.h"

#define ENABLE_DEBUG 0
#include "debug.h"
#define LOG_PREFIX "boot_guard: "
#include "log.h"

/* Consecutive failed boots before the current slot is rolled back. */
#ifndef CONFIG_BOOT_GUARD_MAX_ATTEMPTS
#define CONFIG_BOOT_GUARD_MAX_ATTEMPTS 3
#endif

/* Watchdog reset window. Must comfortably exceed the healthy boot time (from
 * reset until the motion loop first beats), since the watchdog is not kicked
 * before then. */
#ifndef CONFIG_BOOT_GUARD_WDT_MS
#define CONFIG_BOOT_GUARD_WDT_MS 8000
#endif

/* Marks the .noinit state as valid; a power cycle leaves SRAM with a random
 * value here, which reads as a cold boot. */
#define BOOT_GUARD_MAGIC 0x600db007UL

typedef struct
{
    uint32_t magic;
    uint32_t attempts;
} boot_guard_state_t;

/* Retained across warm (watchdog/soft) resets, lost on power-on/BOR. */
static boot_guard_state_t _state __attribute__((section(".noinit")));

volatile uint32_t boot_guard_liveness;

static char _kicker_stack[THREAD_STACKSIZE_DEFAULT];

/* Kick the watchdog only while the motion loop keeps beating; confirm the boot
 * (clear the failed-boot counter) the first time it does. A stuck boot or a
 * stalled loop stops the beats, so the watchdog is left to reset the board. */
static void* _kicker_thread(void* arg)
{
    (void)arg;
    const uint32_t period_ms = CONFIG_BOOT_GUARD_WDT_MS / 3;
    uint32_t last = boot_guard_liveness;
    bool confirmed = false;

    while (1) {
        ztimer_sleep(ZTIMER_MSEC, period_ms);
        uint32_t now = boot_guard_liveness;
        if (now != last) {
            wdt_kick();
            last = now;
            if (!confirmed) {
                _state.attempts = 0;
                confirmed = true;
                LOG_INFO(LOG_PREFIX "boot confirmed healthy\n");
            }
        }
    }
    return NULL;
}

void boot_guard_init(void)
{
    /* A power cycle leaves _state uninitialised: treat as a fresh start. */
    if (_state.magic != BOOT_GUARD_MAGIC) {
        _state.magic = BOOT_GUARD_MAGIC;
        _state.attempts = 0;
    }

    _state.attempts++;
    LOG_INFO(LOG_PREFIX "boot attempt %" PRIu32 "/%u\n", _state.attempts,
             CONFIG_BOOT_GUARD_MAX_ATTEMPTS);

    /* Arm the watchdog first, so even the rollback path below is watchdog
     * protected; it stays un-kicked until the motion loop beats. */
    wdt_setup_reboot(0, CONFIG_BOOT_GUARD_WDT_MS);
    wdt_start();

    if (_state.attempts >= CONFIG_BOOT_GUARD_MAX_ATTEMPTS) {
        int slot = riotboot_slot_current();
        /* Roll back only if the other slot is valid, so the board is never left
         * without a bootable image. */
        if (riotboot_slot_validate(riotboot_slot_other()) == 0) {
            /* Invalidate by erasing the slot's header page. Overwriting the
             * header in place (what riotboot_flashwrite_invalidate does) is
             * rejected on the STM32H5: its flash forbids reprogramming an
             * already-written quadword, and the write buffer alignment differs.
             * Erasing makes the header read 0xFF -> invalid magic -> the
             * bootloader falls back. This is permanent: the slot stays unbootable
             * across a cold boot until it is deliberately reflashed. */
            unsigned page = flashpage_page((void*)riotboot_slot_get_hdr(slot));
            LOG_WARNING(LOG_PREFIX "slot %d failed %" PRIu32 " times, rolling "
                                   "back\n",
                        slot, _state.attempts);
            /* Clear the counter before erasing so a reset during the erase does
             * not relaunch the rollback on every subsequent boot. */
            _state.attempts = 0;
            /* The erase stalls reads of the running bank; mask interrupts so no
             * ISR is fetched from flash mid-erase (the erase routine itself
             * runs from RAM). */
            unsigned irq_state = irq_disable();
            flashpage_erase(page);
            irq_restore(irq_state);
            pm_reboot();
        }
        /* No valid fallback: keep running this slot and start the count over
         * rather than retry the rollback every boot. */
        LOG_WARNING(LOG_PREFIX "no valid fallback slot, staying on slot %d\n", slot);
        _state.attempts = 0;
    }

    thread_create(_kicker_stack, sizeof(_kicker_stack), THREAD_PRIORITY_MAIN - 1,
                  THREAD_CREATE_STACKTEST, _kicker_thread, NULL, "boot_guard");
}
