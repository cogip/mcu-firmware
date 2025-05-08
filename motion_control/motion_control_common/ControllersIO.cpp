#include "motion_control_common/ControllersIO.hpp"
#include "etl/fnv_1.h"

// Compute FNV‑1a 32‑bit hash of a string key
IOKey ControllersIO::hash_key(const etl::string<32>& key) {
    etl::fnv_1a_32 fnv(key.begin(), key.end());
    return fnv.value();
}

// Store value unless key is read-only
void ControllersIO::set(const etl::string<32>& key, const IOValue& value) {
    auto h = hash_key(key);
    if (!readonly_keys_.contains(h)) {
        data_[h] = value;
    }
}

// Mark key as read-only
void ControllersIO::set_readonly(const etl::string<32>& key) {
    readonly_keys_.insert(hash_key(key));
}

// Return stored value or empty optional
OptionalIOValue ControllersIO::get(const etl::string<32>& key) const {
    auto h = hash_key(key);
    auto it = data_.find(h);
    return (it != data_.end()) ? OptionalIOValue{it->second} : OptionalIOValue{};
}

template <typename T>
etl::optional<T> ControllersIO::get_as(const etl::string<32>& key) const {
  if (auto opt = get(key)) {
    if (etl::holds_alternative<T>(*opt)) {
      return etl::get<T>(*opt);
    }
  }
  return {};  // empty optional
}
