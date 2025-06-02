#include "motion_control_common/ControllersIO.hpp"
#include "etl/fnv_1.h"

namespace cogip {

namespace motion_control {

/// Compute FNV-1a 32-bit hash of a string key.
ParamKey ControllersIO::hash_key(etl::string_view key) {
    return etl::fnv_1a_32(key.begin(), key.end()).value();
}

/// Set or update a parameter value. Returns EACCES if the key is read-only.
int ControllersIO::set(etl::string_view key, const ParamValue& value) {
    ParamKey h = hash_key(key);
    if (readonly_keys_.contains(h)) {
        return EACCES;
    }
    data_[h] = value;
    _modified_keys.insert(h);
    return 0;
}

/// Mark a parameter key as read-only.
void ControllersIO::mark_readonly(etl::string_view key) {
    readonly_keys_.insert(hash_key(key));
}

/// Retrieve the raw variant value for a key.
OptionalValue ControllersIO::get(etl::string_view key) const {
    ParamKey h = hash_key(key);
    auto it = data_.find(h);
    return (it != data_.end()) ? OptionalValue{it->second} : OptionalValue{};
}

/// Returns a vector of all ParamKeys that have been written since the last clear_modified().
etl::vector<ParamKey, MAX_PARAMS> ControllersIO::snapshot_modified() const {
    etl::vector<ParamKey, MAX_PARAMS> result;
    result.reserve(_modified_keys.size());
    for (auto it = _modified_keys.begin(); it != _modified_keys.end(); ++it) {
        result.push_back(*it);
    }
    return result;
}

/// @brief Find which keys in `new_keys` also appear in `already_written`.
/// @param new_keys         Vector of keys newly written by one controller subtree.
/// @param already_written  Set of keys written so far by earlier siblings.
/// @return A vector of ParamKey that appear in both.
etl::vector<ParamKey, MAX_PARAMS> ControllersIO::find_collisions(
    const etl::vector<ParamKey, MAX_PARAMS>& new_keys,
                const etl::set<ParamKey, MAX_PARAMS>& already_written)
{
    etl::vector<ParamKey, MAX_PARAMS> collisions;
    collisions.reserve(new_keys.size());

    for (auto h_new : new_keys) {
        if (already_written.contains(h_new)) {
            collisions.push_back(h_new);
        }
    }
    return collisions;
}

/// @brief Compute the set‐difference “after \ before” for two lists of keys.
/// @param before_ctrl All keys present before running a subtree.
/// @param after_ctrl  All keys present after running that subtree.
/// @return A vector containing keys in `after_ctrl` but not in `before_ctrl`.
etl::vector<ParamKey, MAX_PARAMS>
ControllersIO::difference(const etl::vector<ParamKey, MAX_PARAMS>& after_ctrl,
           const etl::vector<ParamKey, MAX_PARAMS>& before_ctrl)
{
    etl::vector<ParamKey, MAX_PARAMS> result;
    result.reserve(after_ctrl.size());

    for (auto h_new : after_ctrl) {
        bool in_before = false;
        for (auto h_old : before_ctrl) {
            if (h_new == h_old) {
                in_before = true;
                break;
            }
       }
        if (!in_before) {
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
