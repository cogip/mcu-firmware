// Copyright (C) 2023 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup    motion_control_common
/// @{
/// @file
/// @brief      Base class for all controllers, meta-controllers and engines
/// @author     Eric Courtois <eric.courtois@gmail.com>
/// @author     Gilles DOFFE <g.doffe@gmail.com>

#pragma once

// System includes
#include <iostream>

// Project includes
#include "etl/vector.h"

namespace cogip {

namespace motion_control {

/// Status of a position to reach
/// - reached:      the position has been reached
/// - intermediate: a transient position has been reached
/// - ongoing:      moving
typedef enum {moving = 0, reached, intermediate_reached} target_pose_status_t;

// Forward declarations
class BaseMetaController;

/// All controllers, filters and metas inherit from this class
class BaseController {
public:
    /// Constructor
    BaseController() : meta_(nullptr) {};

    /// Destructor
    virtual ~BaseController() {};

    /// Get input at given index
    /// return input
    virtual double input (
        size_t index    ///< [in]  index
        ) const = 0;

    /// Set input at given index
    virtual void set_input(
        size_t index,   ///< [in]  index
        double value    ///< [in]  value
        ) = 0;

    /// Get output at given index
    /// return output
    virtual double output (
        size_t index    ///< [in]  index
        ) const = 0;

    /// Set output at given index
    virtual void set_output(
        size_t index,   ///< [in]  index
        double value    ///< [in]  value
        ) = 0;

    /// Get numer of inputs
    /// return number of inputs
    virtual size_t nb_inputs() const = 0;

    /// Get numer of outputs
    /// return number of outputs
    virtual size_t nb_outputs() const = 0;

    /// Controller core method
    virtual void execute() = 0;

    /// Get meta controller to which current controller belongs to
    /// return Meta controller
    BaseMetaController *meta() const { return meta_; };

    /// Set meta controller to which current controller belongs to
    /// return true if set, false otherwise
    virtual bool set_meta(
        BaseMetaController *meta    ///< [in]  meta controller
        );

protected:
    ///  Check if given index is valid regarding number of inputs
    /// return true if index is valid, false otherwise
    bool is_index_valid(
        size_t index    ///< [in]  index
        );

private:
    /// Meta controller to which current controller belongs to
    BaseMetaController *meta_;
};

} // namespace motion_control

} // namespace cogip

/// @}
