#pragma once

#include <type_traits>
#include <iostream>

/// Convert Class Enum member into its underlying integer
template <typename Enumeration>
constexpr std::enable_if_t<std::is_enum<Enumeration>::value,
std::underlying_type_t<Enumeration>> as_number(const Enumeration value)
{
    return static_cast<std::underlying_type_t<Enumeration>>(value);
}

/// Add Class Enum support to iostream << operator
template<typename T>
std::ostream& operator<<(typename std::enable_if<std::is_enum<T>::value, std::ostream>::type& stream, const T& e)
{
    return stream << static_cast<typename std::underlying_type<T>::type>(e);
}

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
