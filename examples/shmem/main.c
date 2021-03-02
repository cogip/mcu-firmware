#include <assert.h>
#include <stdio.h>

#include "shmem.h"
#include "shell_menu.h"

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

int main(void)
{
    puts("\n== shmem example ==");

    /* make set_key command available in all menus */
    const shell_command_t global_commands[] = {
        shmem_set_key_cmd,
        menu_cmd_null
    };
    menu_set_global_commands(global_commands);

    /* Create the main menu */
    shell_menu_t main_menu;

    menu_init(&main_menu, "Main menu");

    /* Add print data command */
    const shell_command_t print_data_cmd = {
        "data", "Print data from the shared memory",
        print_data_cmd_cb
    };
    menu_add_one(&main_menu, &print_data_cmd);

    /* Push main menu */
    menu_enter(&main_menu);

    /* Start shell */
    menu_start();

    return 0;
}
