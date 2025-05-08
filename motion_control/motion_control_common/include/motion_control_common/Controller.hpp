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

/// A controller transforms input data to outputs
/// @tparam INPUT_SIZE  number of inputs
/// @tparam OUTPUT_SIZE number of outputs
/// @tparam ParamsT     controller parameters type

///
/// @tparam ParamsT
template <typename ParamsT, typename IOKeysT>
class Controller : virtual public BaseController {
public:
    /// Constructors
    Controller(
        ParamsT *parameters = nullptr,  ///< [in]  Controller parameters, specific to controller derived classes
        IOKeysT *keys = nullptr,
    ) : BaseController(), parameters_(parameters) {};

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
};

} // namespace motion_control

} // namespace cogip

/// @}
