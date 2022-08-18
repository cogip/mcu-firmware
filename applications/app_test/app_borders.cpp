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

const cogip::obstacles::Polygon &app_get_borders(void)
{
    static obstacles::Polygon borders(border_points);
    return borders;
}

} // namespace app

} // namespace cogip
