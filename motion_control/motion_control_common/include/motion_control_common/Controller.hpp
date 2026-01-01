// Copyright (C) 2023 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup    motion_control_common
/// @{
/// @file
/// @brief      A simple controller
/// @author     Eric Courtois <eric.courtois@gmail.com>
/// @author     Gilles DOFFE <g.doffe@gmail.com>

#pragma once

// Project includes
#include "BaseController.hpp"
#include "etl/vector.h"
#include "utils.hpp"

namespace cogip {

namespace motion_control {

// Extern classes
class BaseMetaController;

/// @brief Base class for typed controllers with parameters and IO key mapping.
///
/// This class transforms inputs to outputs using a `ControllersIO` object.
/// It is templated to allow specific parameter and key types for each
/// controller. Input/output keys are typically defined in a tag structure
/// (e.g., LinearTags, AngularTags), enabling reuse of generic controller logic
/// with different data sets.
///
/// @tparam IOKeysT Type defining the names/keys used to access inputs and
/// outputs.
/// @tparam ParamsT Type defining the controller's parameters.
template <typename IOKeysT, typename ParamsT> class Controller : virtual public BaseController
{
  public:
    /// @brief Constructor for the controller.
    /// @param keys Reference to the IO key definitions used to access
    /// ControllersIO values.
    /// @param parameters Reference to controller parameters.
    /// @param name Optional instance name for identification.
    Controller(const IOKeysT& keys, const ParamsT& parameters, etl::string_view name = "")
        : BaseController(name), keys_(keys), parameters_(parameters){};

    /// Get parameters
    /// return current parameters
    virtual const ParamsT& parameters() const
    {
        return parameters_;
    };

  protected:
    /// Controller IOs keys used by this controller
    const IOKeysT& keys_;

    /// Controller parameters
    const ParamsT& parameters_;
};

} // namespace motion_control

} // namespace cogip

/// @}
