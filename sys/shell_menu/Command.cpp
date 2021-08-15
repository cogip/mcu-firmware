#include "shell_menu/Command.hpp"
#include "shell_menu_private.hpp"

namespace cogip {

namespace shell {

Command::Command(const ::std::string &name, const ::std::string &desc, shell_command_handler_t handler)
    : name_(name), desc_(desc), handler_(handler)
{
    all_commands.insert(this);
}

Command::~Command()
{
    all_commands.erase(this);
}

} // namespace shell

} // namespace cogip
