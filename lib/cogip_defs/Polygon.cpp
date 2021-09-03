#include "cogip_defs/Polygon.hpp"

// System include
#include <algorithm>

namespace cogip {

namespace cogip_defs {

int Polygon::point_index(const Coords &p) const
{
    auto it = std::find(begin(), end(), p);
    if (it != end())
    {
        return it - begin();
    }
    return -1;
}

} // namespace cogip_defs

} // namespace cogip
