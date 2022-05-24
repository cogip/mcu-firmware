#pragma once

#include "obstacles/Polygon.hpp"

namespace cogip {

namespace app {

/// Return a polygon obstacle delimitng the table borders.
const cogip::obstacles::Polygon &app_get_borders(void);

} // namespace app

} // namespace cogip
