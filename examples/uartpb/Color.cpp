#include "Color.hpp"

namespace cogip {

namespace cogip_defs {

const char * get_color_name(Color color)
{
    switch (color)
    {
#define COLOR_MACRO(name, value) case Color::name: return #name;
#include "colors.inc"
#undef COLOR_MACRO
        default:
            return "Undefined color";
    }
}

} // namespace cogip_defs

} // namespace cogip
