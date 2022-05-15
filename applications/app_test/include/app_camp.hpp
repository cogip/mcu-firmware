#pragma once

namespace cogip {

namespace app {

/// Enum for camp colors
enum class CampColor {
    Yellow = 0,
    Purple = 1
};

CampColor app_camp_get_color(bool opposite=false);
void app_camp_set_color(CampColor color);
double app_camp_adapt_distance(double dist);
double app_camp_adapt_angle(double angle);

} // namespace app

} // namespace cogip
