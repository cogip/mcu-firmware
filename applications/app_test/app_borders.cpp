#include "obstacles/Polygon.hpp"
#include "platform.hpp"

namespace cogip {

namespace app {

static const etl::vector<cogip::cogip_defs::Coords, 4> border_points = {
    {
        AVOIDANCE_BORDER_X_MIN,
        AVOIDANCE_BORDER_Y_MIN
    },
    {
        AVOIDANCE_BORDER_X_MAX,
        AVOIDANCE_BORDER_Y_MIN
    },
    {
        AVOIDANCE_BORDER_X_MAX,
        AVOIDANCE_BORDER_Y_MAX
    },
    {
        AVOIDANCE_BORDER_X_MIN,
        AVOIDANCE_BORDER_Y_MAX
    }
};

static cogip::obstacles::Polygon *_borders = nullptr;

const cogip::obstacles::Polygon &app_get_borders(void)
{
    if (! _borders) {
        _borders = new cogip::obstacles::Polygon(border_points);
    }
    return *_borders;
}

} // namespace app

} // namespace cogip
