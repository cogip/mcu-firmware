#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>

/* RIOT includes */
#include "mtd_sdcard.h"
#include "sdcard_spi.h"
#include "sdcard_spi_params.h"
#include "shell.h"

/* include file system header */
#include "fs/fatfs.h"

#define SDCARD_MOUNT_POINT "/sdcard"

/* This is provided by the sdcard_spi driver
 * See drivers/sdcard_spi/sdcard_spi.c */
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
    .vol_idx = 0, /* Low level device that is used by FatFs */
    .fat_fs = {
        .fs_type = FS_EXFAT
    }
};

/* Define the vfs mount point */
static vfs_mount_t flash_mount = {
    .fs = &fatfs_file_system,
    .mount_point = SDCARD_MOUNT_POINT,
    .private_data = &fs_desc,
};

int test_write_vfs(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    const char *test_write_filename = SDCARD_MOUNT_POINT "/vfs.txt";
    const char *line1 = "Write text - line 1\n";
    const char *line2 = "Write text - line 2\n";
    size_t len1 = strlen(line1);
    size_t len2 = strlen(line2);
    int res = 0;

    int fd = vfs_open(test_write_filename, O_RDWR | O_CREAT | O_TRUNC, 0);

    if (fd < 0) {
        printf("Error while trying to create %s (%d)\n", test_write_filename, fd);
        return 1;
    }
    printf("'%s' opened successfully for write\n", test_write_filename);

    res = vfs_write(fd, line1, len1);
    if (res != (ssize_t)len1) {
        printf("Write line1: error (written=%d, expected=%zu)\n", res, len1);
    }
    else {
        printf("Write line1: success\n");
    }

    res = vfs_write(fd, line2, len2);
    if (res != (ssize_t)len2) {
        printf("Write line2: error (written=%d, expected=%zu)\n", res, len2);
    }
    else {
        printf("Write line2: success\n");
    }

    res = vfs_close(fd);
    if (res != 0) {
        printf("File '%s' close: failed (%d)\n", test_write_filename, res);
        return 1;
    }
    printf("File '%s' close: success\n", test_write_filename);

    return 0;
}

int test_write_std(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    const char *test_write_filename = SDCARD_MOUNT_POINT "/std.txt";
    const char *line1 = "Write text - line 1\n";
    const char *line2 = "Write text - line 2\n";
    size_t len1 = strlen(line1);
    size_t len2 = strlen(line2);
    int len = 0;
    int errsv;

    int fd = open(test_write_filename, O_RDWR | O_CREAT | O_TRUNC);

    if (fd < 0) {
        printf("Error while trying to create %s\n", test_write_filename);
        return 1;
    }
    printf("'%s' opened successfully for write\n", test_write_filename);

    len = write(fd, line1, len1);
    if (len != (ssize_t)len1) {
        errsv = errno;
        printf("Write line1: error (written=%d, expected=%zu)\n", len, len1);
        printf("Last errno: %d\n", errsv);
        perror("Last error");
    }
    else {
        printf("Write line1: success\n");
    }

    len = write(fd, line2, len2);
    if (len != (ssize_t)len2) {
        errsv = errno;
        printf("Write line2: error (written=%d, expected=%zu)\n", len, len2);
        printf("Last errno: %d\n", errsv);
        perror("Last error");
    }
    else {
        printf("Write line2: success\n");
    }

    if (close(fd) != 0) {
        errsv = errno;
        printf("File '%s' close: failed\n", test_write_filename);
        printf("Last errno: %d\n", errsv);
        perror("Last error");
        return 1;
    }
    printf("File '%s' close: success\n", test_write_filename);

    return 0;
}

int test_write_newlib(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    const char *test_write_filename = SDCARD_MOUNT_POINT "/newlib.txt";
    const char *line1 = "Write text - line 1\n";
    const char *line2 = "Write text - line 2\n";
    int errsv;

    FILE *f = fopen(test_write_filename, "w+");

    if (f == NULL) {
        printf("Error while trying to create %s\n", test_write_filename);
        return 1;
    }
    printf("'%s' opened successfully for write\n", test_write_filename);

    if (fwrite(line1, 1, strlen(line1), f) != strlen(line1)) {
        printf("Write line1: failed\n");
    }
    else {
        printf("Write line1: success\n");
    }

    if (fwrite(line2, 1, strlen(line2), f) != strlen(line2)) {
        printf("Write line2: failed\n");
    }
    else {
        printf("Write line2: success\n");
    }

    if (fflush(f) != 0) {
        errsv = errno;
        printf("File '%s' flush: failed\n", test_write_filename);
        printf("Last errno: %d\n", errsv);
        perror("Last error");
        fclose(f);
        return 1;
    }
    printf("File '%s' flush: success\n", test_write_filename);

    if (fclose(f) != 0) {
        errsv = errno;
        printf("File '%s' close: failed\n", test_write_filename);
        printf("Last errno: %d\n", errsv);
        perror("Last error");
        return 1;
    }
    printf("File '%s' close: success\n", test_write_filename);

    return 0;
}

static const shell_command_t shell_commands[] = {
    { "test1", "test write vfs", test_write_vfs },
    { "test2", "test write std", test_write_std },
    { "test3", "test write newlib", test_write_newlib },
    { NULL, NULL, NULL }
};

int main(void)
{
    fs_desc.dev = (mtd_dev_t *)&mtd_sdcard_dev;

    int res = vfs_mount(&flash_mount);

    if (res < 0) {
        puts("Error while mounting fatfs");
        return 1;
    }
    puts("fatfs mounted successfully");

    char line_buf[SHELL_DEFAULT_BUFSIZE];

    shell_run(shell_commands, line_buf, SHELL_DEFAULT_BUFSIZE);

    res = vfs_umount(&flash_mount);
    if (res < 0) {
        printf("Error while unmounting %s\n", flash_mount.mount_point);
        return 1;
    }
    printf("%s successfully unmounted\n", flash_mount.mount_point);

    return 0;
}
