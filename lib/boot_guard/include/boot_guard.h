#pragma once

/**
 * @file
 * @brief   Confirmed-boot guard with automatic riotboot rollback
 *
 * Detects a firmware that boots but never becomes healthy (panics or hangs
 * before the motion control loop runs, or a runtime hang of that loop) and,
 * after a few consecutive failed boots, permanently invalidates the current
 * riotboot slot so the bootloader falls back to the other slot.
 *
 * Mechanism:
 *  - a hardware watchdog is armed early and is only kicked while the motion
 *    loop keeps beating (@ref boot_guard_beat), so a stuck boot or a stuck
 *    loop triggers a reset;
 *  - a failed-boot counter lives in a `.noinit` variable, retained across the
 *    warm (watchdog/soft) resets that a crash loop produces but cleared on a
 *    power cycle;
 *  - once the counter reaches the threshold, the current slot's header is
 *    erased in flash (permanent: the slot is never booted again, even after a
 *    cold boot, until it is deliberately reflashed) and the board reboots onto
 *    the other slot.
 *
 * This is a reliability mechanism, not a security one.
 */

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   Liveness counter, bumped by @ref boot_guard_beat.
 *
 * Exposed so the beat can be a header-inlined increment on the hot motion
 * loop path; do not write it directly, call @ref boot_guard_beat.
 */
extern volatile uint32_t boot_guard_liveness;

/**
 * @brief   Arm the guard: account this boot, roll back if needed, start the
 *          watchdog and its liveness-gated kicker.
 *
 * Call once, as early as possible in main(), before any lengthy init. If this
 * boot pushes the failed-boot counter to the threshold and the other slot is
 * valid, the current slot is invalidated and the board reboots (this call does
 * not return in that case).
 */
void boot_guard_init(void);

/**
 * @brief   Signal that the critical (motion) loop made progress.
 *
 * Must be called every iteration of the motion control loop. The watchdog is
 * only kicked while this keeps advancing, so a stalled loop eventually resets
 * the board.
 */
static inline void boot_guard_beat(void)
{
    boot_guard_liveness++;
}

#ifdef __cplusplus
}
#endif
