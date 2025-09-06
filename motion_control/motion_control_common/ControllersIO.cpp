#include "motion_control_common/ControllersIO.hpp"
#include "etl/fnv_1.h"
#include "etl/algorithm.h"
#include <iostream>

namespace cogip {

namespace motion_control {

/// Compute FNV-1a 32-bit hash of a string key.
ParamKey ControllersIO::hash_key(KeyType key) {
    return etl::fnv_1a_32(key.begin(), key.end()).value();
}

/// Set or update a parameter value. Returns EACCES if the key is read-only.
int ControllersIO::set(KeyType key, const ParamValue& value) {
    ParamKey h = hash_key(key);
    if (readonly_keys_.contains(h)) {
        std::cerr << "Cannot set read-only " << std::string(key.data(), key.size()) << std::endl;
        return EACCES;
    }
    if (data_.full() && !data_.contains(h)) {
        std::cerr << "Error: Cannot set " << std::string(key.data(), key.size())
                  << " - parameters storage is full" << std::endl;
        return ENOMEM;
    }
    data_[h] = value;
    if (_modified_keys.full() && !_modified_keys.contains(h)) {
        std::cerr << "Warning: Cannot track modification for " << std::string(key.data(), key.size())
                  << " - modified keys storage is full" << std::endl;
    } else {
        _modified_keys.insert(h);
    }
    return 0;
}

/// Mark a parameter key as read-only.
void ControllersIO::mark_readonly(KeyType key) {
    if (readonly_keys_.full()) {
        std::cerr << "Error: Cannot mark " << std::string(key.data(), key.size())
                  << " as read-only - readonly keys storage is full" << std::endl;
        return;
    }
    readonly_keys_.insert(hash_key(key));
}

/// Reset read-only parameters list.
void ControllersIO::reset_readonly_markers() {
    readonly_keys_.clear();
}

/// Retrieve the raw variant value for a key.
OptionalValue ControllersIO::get(KeyType key) const {
    ParamKey h = hash_key(key);
    auto it = data_.find(h);
    if (it == data_.end()) {
        std::cerr << "Error key not found " << std::string(key.data(), key.size()) << std::endl;
    }
    return (it != data_.end()) ? OptionalValue{it->second} : OptionalValue{};
}

/// Returns a vector of all ParamKeys that have been written since the last clear_modified().
ParamKeyVector ControllersIO::snapshot_modified() const {
    ParamKeyVector result;
    result.reserve(_modified_keys.size());
    for (auto it = _modified_keys.begin(); it != _modified_keys.end(); ++it) {
        if (result.full()) {
            std::cerr << "Warning: snapshot_modified() result vector is full, some keys will be missing" << std::endl;
            break;
        }
        result.push_back(*it);
    }
    return result;
}

/// @brief Find which keys in `new_keys` also appear in `already_written`.
/// @param new_keys         Vector of keys newly written by one controller subtree.
/// @param already_written  Set of keys written so far by earlier siblings.
/// @return A vector of ParamKey that appear in both.
ParamKeyVector ControllersIO::find_collisions(
    const ParamKeyVector& new_keys,
                const etl::set<ParamKey, MAX_PARAMS>& already_written)
{
    ParamKeyVector collisions;
    collisions.reserve(new_keys.size());

    for (auto h_new : new_keys) {
        if (already_written.contains(h_new)) {
            if (collisions.full()) {
                std::cerr << "Warning: find_collisions() result vector is full, some collisions will be missing" << std::endl;
                break;
            }
            collisions.push_back(h_new);
        }
    }
    return collisions;
}

/// @brief Compute the set‐difference “after \ before” for two lists of keys.
/// @param before_ctrl All keys present before running a subtree.
/// @param after_ctrl  All keys present after running that subtree.
/// @return A vector containing keys in `after_ctrl` but not in `before_ctrl`.
ParamKeyVector
ControllersIO::difference(const ParamKeyVector& after_ctrl,
           const ParamKeyVector& before_ctrl)
{
    ParamKeyVector result;
    result.reserve(after_ctrl.size());

    for (auto h_new : after_ctrl) {
        // Define predicate with explicit type to ensure correct TUnaryPredicate
        auto predicate = [h_new](const ParamKey& h_old) -> bool {
            return h_new == h_old;
        };
        bool in_before = etl::any_of(before_ctrl.begin(), before_ctrl.end(), predicate);
        if (!in_before) {
            if (result.full()) {
                std::cerr << "Warning: difference() result vector is full, some keys will be missing" << std::endl;
                break;
            }
            result.push_back(h_new);
        }
    }
    return result;
}


/// Clears the internal “dirty” set, so subsequent sets will be newly recorded.
void ControllersIO::clear_modified() {
    _modified_keys.clear();
}

} // namespace motion_control

} // namespace cogip
