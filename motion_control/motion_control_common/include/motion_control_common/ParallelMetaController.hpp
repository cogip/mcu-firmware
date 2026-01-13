// Copyright (C) 2023 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup    motion_control_common
/// @{
/// @file
/// @brief      Meta-controller to execute some controllers in pseudo parallel
/// @author     Eric Courtois <eric.courtois@gmail.com>
/// @author     Gilles DOFFE <g.doffe@gmail.com>

#pragma once

// RIOT includes
#include "log.h"
#include <inttypes.h>

#include <debug.h>

// Project includes
#include "ControllersIO.hpp"
#include "MetaController.hpp"
#include "etl/deque.h"

namespace cogip {
namespace motion_control {

/// @brief Runs multiple sub-controllers “in parallel” using the same
/// ControllersIO.
/// @details If any ParamKey gets written by ≥2 top‐level controllers, prints a
/// warning.
/// @tparam NB_CONTROLLERS Maximum number of sub-controllers (default: METACONTROLLER_DEFAULT_SIZE).
template <size_t NB_CONTROLLERS = METACONTROLLER_DEFAULT_SIZE>
class ParallelMetaController : public MetaController<NB_CONTROLLERS>
{
  public:
    /// Constructor
    /// @param name Optional instance name for identification
    explicit ParallelMetaController(etl::string_view name = "")
        : MetaController<NB_CONTROLLERS>(name)
    {
    }

    /// @brief Get the type name of this controller
    const char* type_name() const override
    {
        return "ParallelMetaController";
    }

    /// @brief Execute all sub-controllers on the same `io`.
    ///        Detects and warns if the same key is modified by ≥2 controllers.
    /// @param io Shared ControllersIO instance.
    void execute(ControllersIO& io) override
    {
        if (this->controllers_.empty()) {
            LOG_ERROR("Error: no controller added.\n");
            return;
        }

        DEBUG("Execute ParallelMetaController\n");

        // Cumulative set of keys already written by previous controllers.
        etl::set<ParamKey, MAX_PARAMS> cumulative_written;

        // For each top-level controller:
        for (auto ctrl : this->controllers_) {
            auto before_io = io.snapshot_modified();

            // Execute controller
            ctrl->execute(io);

            auto after_io = io.snapshot_modified();

            // Compute difference after/before on inputs/outputs via static helper
            auto just_written = ControllersIO::difference(after_io, before_io);

            // Detect collisions, parallel controllers should not modify the same
            // entry in the inputs/outputs map
            auto collisions = ControllersIO::find_collisions(just_written, cumulative_written);
            for (auto h : collisions) {
                LOG_WARNING("Key hash %" PRIu32
                            " was already written by another parallel controller\n",
                            static_cast<uint32_t>(h));
            }

            // Merge
            for (auto h_new : just_written) {
                cumulative_written.insert(h_new);
            }
        }
    }
};

} // namespace motion_control

} // namespace cogip

/// @}
