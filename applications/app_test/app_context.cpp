#include "app_context.hpp"

namespace cogip {

namespace app {

cogip::app::Context *_context = nullptr;

Context::Context()
{
}

Context &app_get_context(void)
{
    static Context context;
    return context;
}

} // namespace app

} // namespace cogip
