#include "obstacles/Polygon.hpp"
#include "platform.hpp"

static const std::list<cogip::cogip_defs::Coords> border_points = {
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

cogip::obstacles::Polygon *_borders = nullptr;

const cogip::obstacles::Polygon &app_get_borders(void)
{
    if (! _borders) {
        _borders = new cogip::obstacles::Polygon(&border_points);
    }
    return *_borders;
}
