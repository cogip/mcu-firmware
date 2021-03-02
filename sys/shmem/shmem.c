#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/shm.h>

#include "shell.h"

#include "shmem.h"
#include "shmem_params.h"

/* Shared memory key used to communicate with the simulator */
static int shmem_key = 0;

/* Shared memory structure shared by the simulator */
static shmem_data_t *shmem_data = NULL;

/* Command to add to the shell */
const shell_command_t shmem_set_key_cmd = {
    "set_shmem_key", "Set shared memory key to communicate with simulator",
    shmem_set_key_cmd_cb
};

void shmem_set_key(int key)
{
    /* Detach previous shared memory if any */
    if (shmem_key > 0) {
        shmdt(shmem_data);
    }

    shmem_key = key;

    /* Attach new shared memory if SHM key is set */
    if (shmem_key > 0) {
        int shmid = shmget(shmem_key, sizeof(shmem_data_t), 0);
        shmem_data = (shmem_data_t *)shmat(shmid, (void *)0, 0);
    }
}

int shmem_get_key(void)
{
    return shmem_key;
}

const shmem_data_t *shmem_get_data(void)
{
    return shmem_data;
}

int shmem_set_key_cmd_cb(int argc, char **argv)
{
    /* Check arguments */
    if (argc != 2) {
        puts("Error: bad number of arguments.");
        return EXIT_FAILURE;
    }

    /* Get new SHM key generated by the simulator */
    shmem_set_key(atoi(argv[1]));

    return EXIT_SUCCESS;
}
