// Copyright (C) 2021 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup     lib_cogip_defs
/// @{
/// @file
/// @brief       Polygon class declaration
/// @author      Eric Courtois <eric.courtois@gmail.com>

#pragma once

// System includes
#include "etl/vector.h"

// Project includes
#include "cogip_defs/Coords.hpp"

namespace cogip {

namespace cogip_defs {

/// A polygon defined by a list of coordinates
template <size_t N> class Polygon : public etl::vector<Coords, N>
{
  public:
    /// Find a point in this polygon and return its index.
    /// @return point index if found, -1 otherwise
    int point_index(const Coords& p ///< [in] point to find
    ) const
    {
        auto it = std::find(this->begin(), this->end(), p);
        if (it != this->end()) {
            return it - this->begin();
        }
        return -1;
    };
};

} // namespace cogip_defs

} // namespace cogip

/// @}
