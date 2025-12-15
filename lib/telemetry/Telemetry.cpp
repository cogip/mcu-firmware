// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

#include "telemetry/Telemetry.hpp"

namespace cogip {

namespace telemetry {

void Telemetry::init(cogip::canpb::CanProtobuf& canpb, canpb::uuid_t uuid)
{
    canpb_ = &canpb;
    uuid_ = uuid;
    initialized_ = true;
}

void Telemetry::enable()
{
    enabled_ = true;
}

void Telemetry::disable()
{
    enabled_ = false;
}

bool Telemetry::is_enabled()
{
    return enabled_;
}

} // namespace telemetry

} // namespace cogip
