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
/// @tparam INPUT_SIZE
/// @tparam OUTPUT_SIZE
template <size_t INPUT_SIZE, size_t OUTPUT_SIZE, typename ParamsT>
class Controller : virtual public BaseController {
public:
    /// Constructors
    Controller(
        ParamsT *parameters = nullptr   ///< [in]  Controller parameters, specific to controller derived classes
    ) : BaseController(), parameters_(parameters) {};

    /// Get input at given index
    /// return         input at given index, max float value if error
    float input(
        size_t index    ///< [in]  input index
        ) const override {
        if (index >= INPUT_SIZE) {
            COGIP_DEBUG_COUT("Error: not enough values.");
            return std::numeric_limits<float>::max();
        }
        return this->inputs_[index];
    };

    /// Set input at given index
    void set_input(
        size_t index,   ///< [in]  input index
        float value    ///< [in]  value to set at given index
        ) override {
        if (index >= INPUT_SIZE) {
            COGIP_DEBUG_COUT("Error: not enough input values.");
            return;
        }
        this->inputs_[index] = value;
    };

    /// Get output at given index
    /// return         output at given index, max float value if error
    float output(
        size_t index    ///< [in]  output index
        ) const override {
        if (index >= OUTPUT_SIZE) {
            COGIP_DEBUG_COUT("Error: cannot get output at index " << index << ", not enough output values(" << OUTPUT_SIZE << ").");
            return std::numeric_limits<float>::max();
        }
        return this->outputs_[index];
    };

    /// Set output at given index
    void set_output(
        size_t index,   ///< [in]  output index
        float value    ///< [in]  value to set at given index
        ) override {
        if (index >= OUTPUT_SIZE) {
            COGIP_DEBUG_COUT("Error: cannot set output at index " << index << ", not enough output values(" << OUTPUT_SIZE << ").");
            return;
        }
        this->outputs_[index] = value;
    };

    /// Get number of inputs
    /// return number of inputs
    size_t nb_inputs() const override { return INPUT_SIZE; };

    /// Get number of outputs
    /// return number of outputs
    size_t nb_outputs() const override { return OUTPUT_SIZE; };

    /// Get parameters
    /// return current parameters
    virtual const ParamsT* parameters(
        ) { return parameters_; };

    /// Set new parameters
    virtual void set_parameters(
        const ParamsT *new_params   ///< [in]  new controller parameters
        ) { parameters_ = new_params; };

protected:
    /// Inputs vector
    etl::vector<float, INPUT_SIZE> inputs_;

    /// outputs vector
    etl::vector<float, OUTPUT_SIZE> outputs_;

    /// Controller parameters
    const ParamsT *parameters_;
};

} // namespace motion_control

} // namespace cogip

/// @}
