#include "app.hpp"
#include "platform.hpp"

int main(void)
{
    pf_init();
    app_init();

    pf_init_tasks();

    return 0;
}
