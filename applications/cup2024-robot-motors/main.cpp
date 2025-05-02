#include "app.hpp"
#include "platform.hpp"

int main(void)
{
    pf_init();
    cogip::app::app_init();

    pf_init_tasks();

    return 0;
}
