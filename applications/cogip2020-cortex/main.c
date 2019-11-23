#include <stdio.h>

#include "app.h"
#include "platform.h"

int main(void)
{
    pf_init();
    app_init();

    pf_init_tasks();
    app_init_tasks();

    return 0;
}
