#include "shell_menu/Menu.hpp"
#include "shell_menu_private.hpp"

// System includes
#include <cassert>
#include <iostream>

#include "PB_Menu.hpp"

namespace cogip {

namespace shell {

std::ostream& operator << (std::ostream& os, const etl::istring& str)
{
    os << str.c_str();
    return os;
}

/// Protobuf message type. Shortcut for original template type.
using PB_Message = PB_Menu<
    COMMAND_NAME_MAX_LENGTH, NB_SHELL_COMMANDS,
    COMMAND_NAME_MAX_LENGTH, COMMAND_DESC_MAX_LENGTH
    >;

PB_Message pb_message_;           ///< Protobuf message describing this menu.

// Callback executed to enter a sub-menu
static int _cmd_enter_sub_menu(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    auto it = all_menus().find(argv[0]);
    if (it != all_menus().end()) {
        it->second->enter();
    }

    return EXIT_SUCCESS;
}

// menu methods
Menu::Menu(
    const etl::string<COMMAND_NAME_MAX_LENGTH> &name,
    const etl::string<COMMAND_NAME_MAX_LENGTH> &cmd,
    Menu *parent, func_cb_t enter_cb) : name_(name), cmd_(cmd), parent_(parent)
{
    enter_cb_ = enter_cb;

    if (cmd != "") {
        assert(all_menus().count(cmd) == 0);
        all_menus()[cmd] = this;
    }

    if (parent) {
        enter_cmd_ = { cmd, name, _cmd_enter_sub_menu };
        parent->push_back(&enter_cmd_);
    }
}

void Menu::enter(void)
{
    const Menu *previous_menu = current_menu;
    current_menu = this;

    assert (global_commands.size() + current_menu->size() <= NB_SHELL_COMMANDS);

    size_t i = 0;
    for (auto cmd: global_commands) {
        current_commands[i].name = cmd->name().c_str();
        current_commands[i].desc = cmd->desc().c_str();
        current_commands[i].handler = cmd->handler();
        i++;
    }

    for (auto cmd: *current_menu) {
        current_commands[i].name = cmd->name().c_str();
        current_commands[i].desc = cmd->desc().c_str();
        current_commands[i].handler = cmd->handler();
        i++;
    }

    for (; i < NB_SHELL_COMMANDS; i++) {
        current_commands[i].name = nullptr;
        current_commands[i].desc = nullptr;
        current_commands[i].handler = nullptr;
        i++;
    }

    COGIP_DEBUG_COUT("Enter shell menu: " << current_menu->name());

#ifdef MODULE_CANPB
    send_pb_message();
#endif

    if (current_menu != previous_menu && enter_cb_) {
        enter_cb_();
    }
}

void Menu::update_pb_message(void)
{
    pb_message_.clear();
    pb_message_.mutable_name() = name_.c_str();

    for (auto cmd: global_commands) {
        cmd->update_pb_message();
        pb_message_.add_entries(cmd->pb_message());
    }

    for (auto cmd: *current_menu) {
        cmd->update_pb_message();
        pb_message_.add_entries(cmd->pb_message());
    }
}

#ifdef MODULE_CANPB
void Menu::send_pb_message(void)
{
    if (can_protobuf) {
        update_pb_message();
        can_protobuf->send_message(menu_uuid, &pb_message_);
    }
}
#endif

} // namespace shell

} // namespace cogip
