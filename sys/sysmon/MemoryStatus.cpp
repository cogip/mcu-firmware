
// System includes

// RIOT includes

// Project includes
#include "sysmon/MemoryStatus.hpp"

namespace cogip {

namespace sysmon {

MemoryStatus::MemoryStatus() {
    size_ = 0;
    used_ = 0;
}

void MemoryStatus::update_pb_message(void) {
    pb_message_.set_memory_size(size_);
    pb_message_.set_memory_used(used_);
}

} // namespace sysmon

} // namespace cogip
