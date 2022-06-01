#include "app_camp.hpp"

namespace cogip {

namespace app {

static CampColor _camp_color = CampColor::Yellow;

CampColor app_camp_get_color(bool opposite)
{
    if (! opposite) {
        return _camp_color;
    }
    else {
        return _camp_color == CampColor::Yellow ? CampColor::Purple : CampColor::Yellow;
    }
}

void app_camp_set_color(CampColor color)
{
    _camp_color = color;
}

double app_camp_adapt_distance(double dist)
{
    return _camp_color == CampColor::Yellow ? dist : -dist;
}

double app_camp_adapt_angle(double angle)
{
    return _camp_color == CampColor::Yellow ? angle : ((int)(180 - angle)) % 360;
}

} // namespace app

} // namespace cogip
