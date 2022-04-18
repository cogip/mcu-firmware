// Module includes
#include "tracefd/tracefd.hpp"
#include "tracefd_private.hpp"

// System includes
#include <cstdio>

#ifdef TRACEFD_SDCARD_SPI_DEV

// RIOT includes
#include "mtd_sdcard.h"
#include "sdcard_spi.h"
#include "sdcard_spi_params.h"
#include "fs/fatfs.h"

// This is provided by the sdcard_spi driver
// See RIOT/drivers/sdcard_spi/sdcard_spi.c
extern sdcard_spi_t sdcard_spi_devs[ARRAY_SIZE(sdcard_spi_params)];

// Configure MTD device for the first SD card
static mtd_sdcard_t mtd_sdcard_dev = {
    .base = {
        .driver = &mtd_sdcard_driver,
        .sector_count = 0,
        .pages_per_sector = 0,
        .page_size = 0
    },
    .sd_card = &sdcard_spi_devs[TRACEFD_SDCARD_SPI_DEV],
    .params = &sdcard_spi_params[TRACEFD_SDCARD_SPI_DEV]
};

// File system specific descriptor
static fatfs_desc_t fs_desc = {
    .fat_fs = {
        .fs_type = FS_EXFAT,
        .pdrv = 0,
        .n_fats = 0,
        .wflag = 0,
        .fsi_flag = 0,
        .id = 0,
        .n_rootdir = 0,
        .csize = 0,
        .last_clst = 0,
        .free_clst = 0,
        .n_fatent = 0,
        .fsize = 0,
        .volbase = 0,
        .fatbase = 0,
        .dirbase = 0,
        .database = 0,
        .winsect = 0,
        .win = {}
    },
    .vol_idx = 0,     // Low level device that is used by FatFs
    .abs_path_str_buff = {}
};

// Declare mtd devices (for use within diskio layer of fatfs)
// Declared as `extern` in `RIOT/pkg/fatfs/fatfs_diskio/mtd/mtd_diskio.c`
mtd_dev_t *fatfs_mtd_devs[FF_VOLUMES];

// Define the vfs mount point
static vfs_mount_t flash_mount = {
    .list_entry = {},
    .fs = &fatfs_file_system,
    .mount_point = TRACEFD_ROOT_DIR,
    .mount_point_len = 0,
    .open_files = {},
    .private_data = &fs_desc
};
#endif // TRACEFD_SDCARD_SPI_DEV

namespace cogip {

namespace tracefd {

bool init_root_dir(void)
{
    int res = 1;
#ifdef TRACEFD_SDCARD_SPI_DEV
    fatfs_mtd_devs[fs_desc.vol_idx] = (mtd_dev_t *)&mtd_sdcard_dev;
    res = vfs_mount(&flash_mount);
    ::printf("res = %d\n", res);
#endif // TRACEFD_SDCARD_SPI_DEV
    return (res == 0);
}

} // namespace tracefd

} // namespace cogip
