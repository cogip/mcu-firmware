#include "app.hpp"
#include "platform.hpp"

#include "log.h"

int main(void)
{
    LOG_INFO("FW version: %s\n", MCU_FIRMWARE_BUILD_VERSION_STRING);

    pf_init();
    cogip::app::app_init();

    pf_init_tasks();

    return 0;
}
