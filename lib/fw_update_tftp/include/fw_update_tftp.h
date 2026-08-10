#pragma once

/**
 * @file
 * @brief   Ethernet firmware-update service (TFTP server -> riotboot slot)
 *
 * Receives a new firmware image over TFTP (the host pushes it with a
 * `tftp put`) and writes it into the *inactive* riotboot slot using
 * riotboot_flashwrite. The riotboot magic number is held back until the
 * transfer completes, so an interrupted transfer leaves the slot invalid and
 * the bootloader falls back to the previous image. On success the board
 * reboots and riotboot boots the newest valid slot.
 *
 * No signature or encryption is used: this is a reliability mechanism
 * (never boot a half-flashed bank), not a security one.
 */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   Assign the static IPv4 address and start the TFTP update server.
 *
 * Must be called once after the network stack is up (e.g. from
 * pf_init_tasks()). The address is 192.168.0.(100 + ROBOT_ID) unless
 * overridden via FW_UPDATE_IP_LAST_OCTET.
 */
void fw_update_tftp_init(void);

#ifdef __cplusplus
}
#endif
