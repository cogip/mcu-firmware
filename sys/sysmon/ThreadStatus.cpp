
// System includes

// RIOT includes

// Project includes
#include "sysmon/ThreadStatus.hpp"

namespace cogip {

namespace sysmon {

void ThreadStatus::update_pb_message() {
    stack_status_.update_pb_message();
    pb_message_.mutable_stack_status() = stack_status_.pb_message();

    pb_message_.set_pid(pid_);
    pb_message_.mutable_name() = name_.c_str();
    pb_message_.set_loops(loops_);
    pb_message_.set_overshots(overshots_);
}

} // namespace sysmon

} // namespace cogip
