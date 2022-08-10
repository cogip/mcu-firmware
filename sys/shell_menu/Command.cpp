#include "shell_menu/Command.hpp"
#include "shell_menu_private.hpp"

namespace cogip {

namespace shell {

Command::Command(
    const etl::string<COMMAND_NAME_MAX_LENGTH> &name,
    const etl::string<COMMAND_DESC_MAX_LENGTH> &desc,
    shell_command_handler_t handler
    ) : name_(name), desc_(desc), handler_(handler)
{
    all_commands().insert(this);
}

Command::~Command()
{
    all_commands().erase(this);
}

void Command::update_pb_message(void)
{
    pb_message_.clear();
    pb_message_.mutable_cmd() = name_.c_str();
    pb_message_.mutable_desc() = desc_.c_str();
}

} // namespace shell

} // namespace cogip
