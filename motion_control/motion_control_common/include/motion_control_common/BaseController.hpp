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
#include "ControllersIO.hpp"
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
    BaseController() : meta_(nullptr){};

    /// Destructor
    virtual ~BaseController(){};

    /// Controller core method
    /// @param io Controllers input/output datas shared accross controllers
    virtual void execute(ControllersIO& io) = 0;

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

  private:
    /// Meta controller to which current controller belongs to
    BaseMetaController* meta_;
};

} // namespace motion_control

} // namespace cogip

/// @}
