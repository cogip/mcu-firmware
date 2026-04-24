#include "app.hpp"
#include "platform.hpp"

#include "log.h"

int main(void)
{
    LOG_INFO("FW version: %s\n", MCU_FIRMWARE_BUILD_VERSION_STRING);
    LOG_INFO("Robot ID: %d\n", ROBOT_ID);

    pf_init();
    app_init();

    pf_init_tasks();

    return 0;
}
