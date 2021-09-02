#include "cogip_defs/Coords.hpp"

// System includes
#include <cmath>

namespace cogip {

namespace cogip_defs {

double Coords::distance(const Coords &dest) const
{
    return sqrt((dest.x_ - x_) * (dest.x_ - x_)
                + (dest.y_ - y_) * (dest.y_ - y_));
}

bool Coords::on_segment(const Coords &a, const Coords &b) const
{
    bool res = false;

    if ((b.x() - a.x()) / (b.y() - a.y()) == (b.x() - x_) / (b.y() - y_)) {
        if (a.x() < b.x()) {
            if ((x_ < b.x()) && (x_ > a.x())) {
                res = true;
            }
        }
        else {
            if ((x_ < a.x()) && (x_ > b.x())) {
                res = true;
            }
        }
    }
    return res;
}

} // namespace cogip_defs

} // namespace cogip
