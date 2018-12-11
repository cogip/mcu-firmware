#include <stdio.h>

#include "platform.h"
#include "platform_task.h"

int main(void)
{
    mach_setup();

    mach_tasks_init();

    return 0;
}
