// Copyright (C) 2023 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup     lib_pid
/// @{
/// @file
/// @brief       PID implementation
/// @author      Gilles DOFFE <g.doffe@gmail.com>

#pragma once

#include <etl/limits.h>

#include "PB_Pid.hpp"

namespace cogip {

namespace pid {

/// PID
class PID {
public:
    /// Constructor.
    PID(
        float kp = 0.0,                                                ///< [in] proportional gain
        float ki = 0.0,                                                ///< [in] integral gain
        float kd = 0.0,                                                ///< [in] derivative gain
        float integral_term_limit = etl::numeric_limits<float>::max() ///< [in] integral term limit
    ) : kp_(kp), ki_(ki), kd_(kd), integral_term_(0), integral_term_limit_(integral_term_limit), previous_error_(0) {};

    /// Return proportional gain.
    float kp() const { return kp_; };

    /// Return integral gain.
    float ki() const { return ki_; };

    /// Return derivative gain.
    float kd() const { return kd_; };

    /// Return integral term.
    float integral_term() const { return integral_term_; };

    /// Return integral term limit.
    float integral_term_limit() const { return integral_term_limit_; };

    /// Set proportional gain.
    void set_kp(
        float kp   ///< [in] new proportional gain
        ) { kp_ = kp; };

    /// Set integral gain.
    void set_ki(
        float ki   ///< [in] new integral gain
        ) { ki_ = ki; };

    /// Set derivative gain.
    void set_kd(
        float kd   ///< [in] new derivative gain
        ) { kd_ = kd; };

    /// Set integral term limit.
    void set_integral_term_limit(
        float integral_term_limit  ///< [in] new integral term limit
        ) { integral_term_limit_ = integral_term_limit; };

    /// Return previous error.
    float previous_error() const { return previous_error_; };

    /// Reset integral term and previous_error.
    void reset() { integral_term_ = 0; previous_error_ = 0; }

    /// Compute PID.
    float compute(float error);

    /// Initialize the object from a Protobuf message.
    void pb_read(
        const PB_Pid &pid  ///< [in] Protobuf message to read
        ) {
        kp_ = pid.get_kp();
        ki_ = pid.get_ki();
        kd_ = pid.get_kd();
        integral_term_limit_ = pid.get_integral_term_limit();
    };

    /// Copy data to Protobuf message.
    void pb_copy(
        PB_Pid &pid         ///< [out] Protobuf message to fill
        ) const {
        pid.set_kp(kp_);
        pid.set_ki(ki_);
        pid.set_kd(kd_);
        pid.set_integral_term(integral_term_);
        pid.set_integral_term_limit(integral_term_limit_);
        pid.set_previous_error(previous_error_);
    };

private:
    float kp_;                     ///< proportional gain
    float ki_;                     ///< integral gain
    float kd_;                     ///< derivative gain
    float integral_term_;          ///< error sum
    float integral_term_limit_;    ///< error sum maximum limit
    float previous_error_;         ///< previous sum
};

} // namespace pid

} // namespace cogip

/// @}
