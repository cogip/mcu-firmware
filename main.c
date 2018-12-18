#include <stdio.h>

#include "platform.h"

int main(void)
{
    mach_setup();

    mach_tasks_init();

    return 0;
}
