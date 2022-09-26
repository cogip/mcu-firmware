
// System includes

// RIOT includes

// Project includes
#include "sysmon/ThreadStatus.hpp"

namespace cogip {

namespace sysmon {

ThreadStatus::ThreadStatus() : MemoryStatus() {
    pid_ = 0;
}

void ThreadStatus::update_pb_message(void) {
    MemoryStatus::update_pb_message();
    pb_message_.mutable_stack_status() = MemoryStatus::pb_message();

    pb_message_.set_pid(pid_);
    pb_message_.mutable_name() = name_.c_str();
}

} // namespace sysmon

} // namespace cogip
