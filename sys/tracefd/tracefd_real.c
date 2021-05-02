/* RIOT includes */
#include "mtd_sdcard.h"
#include "sdcard_spi.h"
#include "sdcard_spi_params.h"
#include "fs/fatfs.h"

/* Module includes */
#include "tracefd.h"
#include "tracefd_private.h"

/* This is provided by the sdcard_spi driver
 * See RIOT/drivers/sdcard_spi/sdcard_spi.c */
extern sdcard_spi_t sdcard_spi_devs[ARRAY_SIZE(sdcard_spi_params)];

/* Configure MTD device for the first SD card */
static mtd_sdcard_t mtd_sdcard_dev = {
    .base = {
        .driver = &mtd_sdcard_driver
    },
    .sd_card = &sdcard_spi_devs[0],
    .params = &sdcard_spi_params[0],
};

/* File system specific descriptor */
static fatfs_desc_t fs_desc = {
    .vol_idx = 0,     /* Low level device that is used by FatFs */
    .fat_fs = {
        .fs_type = FS_EXFAT
    }
};

/* Declare mtd devices (for use within diskio layer of fatfs) */
/* Declared as `extern` in `RIOT/pkg/fatfs/fatfs_diskio/mtd/mtd_diskio.c` */
mtd_dev_t *fatfs_mtd_devs[FF_VOLUMES];

/* Define the vfs mount point */
static vfs_mount_t flash_mount = {
    .fs = &fatfs_file_system,
    .mount_point = TRACEFD_ROOT_DIR,
    .private_data = &fs_desc,
};

void tracefd_init_root_dir(void)
{
    fatfs_mtd_devs[fs_desc.vol_idx] = (mtd_dev_t *)&mtd_sdcard_dev;
    int res = vfs_mount(&flash_mount);
    assert(res >= 0);
}
