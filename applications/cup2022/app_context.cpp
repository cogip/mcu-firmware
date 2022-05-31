#include "app_context.hpp"

namespace cogip {

namespace app {

cogip::app::Context *_context = nullptr;

Context::Context()
{
}

Context &app_get_context(void)
{
    if (! _context) {
        _context = new Context();
    }
    return *_context;
}

} // namespace app

} // namespace cogip
