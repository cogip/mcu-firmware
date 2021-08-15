#include "shell_menu/Menu.hpp"
#include "shell_menu_private.hpp"

// System includes
#include <cassert>

// Project includes
#include "tracefd/tracefd.hpp"

namespace cogip {

namespace shell {

// Callback executed to enter a sub-menu
static int _cmd_enter_sub_menu(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    auto it = all_menus.find(argv[0]);
    if (it != all_menus.end()) {
        it->second->enter();
    }

    return EXIT_SUCCESS;
}

// menu methods
Menu::Menu(
    const std::string &name, const std::string &cmd,
    Menu *parent, func_cb_t enter_cb) : name_(name), cmd_(cmd), parent_(parent)
{
    enter_cb_ = enter_cb;

    if (cmd != "") {
        assert(all_menus.count(cmd) == 0);
        all_menus[cmd] = this;
    }

    std::string menu_desc = "Enter " + name;
    if (parent) {
        parent->push_back(new Command(cmd, menu_desc, _cmd_enter_sub_menu));
    }
}

void Menu::enter(void) const
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

    cogip::tracefd::out.logf("Enter shell menu: %s", current_menu->name().c_str());

    if (current_menu != previous_menu && enter_cb_) {
        enter_cb_();
    }
}

} // namespace shell

} // namespace cogip
