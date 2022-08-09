#include <cassert>
#include <cstdio>

#include "shmem.h"
#include "shell_menu/shell_menu.hpp"

static int print_data_cmd_cb(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    const shmem_data_t *shared_data = shmem_get_data();
    assert(shared_data);

    printf("Data in shared memory:\n");
    printf("  - data.even =");
    for (int i = 0; i < NB_DATA; i++) {
        printf(" %u", shared_data->even[i]);
    }
    printf("\n");
    printf("  - data.odd  =");
    for (int i = 0; i < NB_DATA; i++) {
        printf(" %u", shared_data->odd[i]);
    }
    printf("\n");

    return 0;
}

static cogip::shell::Command _cmd_data = { "data", "Print data from the shared memory", print_data_cmd_cb };
static cogip::shell::Command _cmd_shmem = SHMEM_SET_KEY_CMD;

int main(void)
{
    puts("\n== shmem example ==");

    // make set_key command available in all menus
    cogip::shell::root_menu().push_back(&_cmd_shmem);

    // Add print data command
    cogip::shell::root_menu().push_back(&_cmd_data);

    // Start shell
    cogip::shell::start();

    return 0;
}
