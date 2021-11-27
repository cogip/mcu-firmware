#pragma once

namespace cogip {

namespace cogip_defs {

enum class Color {
#define COLOR_MACRO(name, value) name = value,
#include "colors.inc"
#undef COLOR_MACRO
};

const char * get_color_name(Color color);

} // namespace cogip_defs

} // namespace cogip
