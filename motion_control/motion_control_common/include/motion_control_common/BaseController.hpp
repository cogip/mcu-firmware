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
#include <cstdio>

// Project includes
#include "ControllersIO.hpp"
#include "etl/string.h"
#include "etl/string_view.h"
#include "etl/vector.h"

namespace cogip {

namespace motion_control {

// Forward declarations
class BaseMetaController;

/// All controllers, filters and metas inherit from this class
class BaseController
{
  public:
    /// Constructor
    /// @param name Optional instance name for identification
    explicit BaseController(etl::string_view name = "") : meta_(nullptr), name_(name){};

    /// Destructor
    virtual ~BaseController(){};

    /// Controller core method
    /// @param io Controllers input/output datas shared accross controllers
    virtual void execute(ControllersIO& io) = 0;

    /// Reset controller internal state
    /// Called when changing target to reinitialize all internal states.
    /// Default implementation does nothing - override in controllers with internal state.
    virtual void reset() {}

    /// Get the type name of this controller (class name)
    /// @return Type name string
    virtual const char* type_name() const
    {
        return "BaseController";
    }

    /// Get the instance name of this controller
    /// @return Instance name string view
    etl::string_view name() const
    {
        return name_;
    }

    /// Set the instance name of this controller
    /// @param name New instance name
    void set_name(etl::string_view name)
    {
        name_ = name;
    }

    /// Dump the controller hierarchy to stdout as ASCII tree
    /// @param indent Current indentation level
    /// @param is_last Whether this is the last child at current level
    /// @param prefix Prefix string for tree drawing
    /// @param counter Execution order counter (incremented for leaf controllers)
    virtual void dump(int indent = 0, bool is_last = true, const char* prefix = "",
                      int* counter = nullptr) const
    {
        // Print tree branch
        if (indent > 0) {
            printf("%s%s", prefix, is_last ? "└── " : "├── ");
        }

        // Print execution order number for leaf controllers
        if (counter) {
            (*counter)++;
            printf("[%d] ", *counter);
        }

        // Print type and name
        if (name_.empty()) {
            printf("%s\n", type_name());
        } else {
            printf("%s: %.*s\n", type_name(), static_cast<int>(name_.size()), name_.data());
        }
    }

    /// Get meta controller to which current controller belongs to
    /// return Meta controller
    BaseMetaController* meta() const
    {
        return meta_;
    };

    /// Set meta controller to which current controller belongs to
    /// return true if set, false otherwise
    virtual bool set_meta(BaseMetaController* meta ///< [in]  meta controller
    );

  protected:
    /// Meta controller to which current controller belongs to
    BaseMetaController* meta_;

    /// Instance name for identification
    etl::string_view name_;
};

} // namespace motion_control

} // namespace cogip

/// @}
