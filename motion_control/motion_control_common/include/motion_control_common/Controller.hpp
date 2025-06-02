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
// class ControllerParameters;
class BaseMetaController;

/// @brief Base class for typed controllers with parameters and IO key mapping.
///
/// This class transforms inputs to outputs using a `ControllersIO` object.
/// It is templated to allow specific parameter and key types for each controller.
/// Input/output keys are typically defined in a tag structure (e.g., LinearTags, AngularTags),
/// enabling reuse of generic controller logic with different data sets.
///
/// @tparam ParamsT Type defining the controller’s parameters.
/// @tparam IOKeysT Type defining the names/keys used to access inputs and outputs.
template <typename ParamsT, typename IOKeysT>
class Controller : virtual public BaseController {
public:
    /// @brief Constructor for the controller.
    /// @param parameters Pointer to controller parameters (can be nullptr if unused).
    /// @param keys Pointer to the IO key definitions used to access ControllersIO values.
    Controller(
        ParamsT* parameters = nullptr,
        const IOKeysT* keys = nullptr
    ) : BaseController(), parameters_(parameters), keys_(keys) {};

    /// Get parameters
    /// return current parameters
    virtual const ParamsT* parameters(
        ) { return parameters_; };

    /// Set new parameters
    virtual void set_parameters(
        const ParamsT *new_params   ///< [in]  new controller parameters
        ) { parameters_ = new_params; };

protected:
    /// Controller parameters
    const ParamsT *parameters_;

    /// Controller IOs keys used by this controller
    const IOKeysT* keys_;
};

} // namespace motion_control

} // namespace cogip

/// @}
