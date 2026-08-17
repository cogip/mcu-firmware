#include "app.hpp"
#include "platform.hpp"

#include "log.h"

#ifdef MODULE_BOOT_GUARD
#include "boot_guard.h"
#endif

int main(void)
{
#ifdef MODULE_BOOT_GUARD
    /* Account this boot and arm the watchdog before any lengthy init, so a
     * firmware that hangs during init is caught and eventually rolled back. */
    boot_guard_init();
#endif

    LOG_INFO("FW version: %s\n", MCU_FIRMWARE_BUILD_VERSION_STRING);
    LOG_INFO("Robot ID: %d\n", ROBOT_ID);

    pf_init();
    app_init();

    pf_init_tasks();

    return 0;
}
