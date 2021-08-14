#include "shell_menu.hpp"

// System includes
#include <cassert>
#include <set>
#include <map>

// Project includes
#include "tracefd.hpp"

namespace cogip {

namespace shell {

menu root_menu("Main", "");                     /// Root menu

// Definition of static variables
static std::map<std::string, menu *> all_menus; /// Map containings all menus indexed by cmd
static std::set<command *> all_commands;        /// All commands
static std::list<command *> global_commands;    /// Global commands, available in all menus
static const menu *current_menu = nullptr;      /// Pointer to the current menu

// Shell commands used by RIOT shell module.
// It is updated each a menu is entered or exited.
static shell_command_t current_commands[NB_SHELL_COMMANDS];

// Callbacks for default global commands
static int _display_json_help(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    cogip::tracefd::out.lock();
    cogip::tracefd::out.printf("{\"name\":\"%s\",\"entries\":[", current_menu->name().c_str());
    shell_command_t *cmd = current_commands;
    for (; cmd->name != NULL; cmd++) {
        if (cmd != current_commands) {
            cogip::tracefd::out.printf(",");
        }
        cogip::tracefd::out.printf(
            "{\"cmd\":\"%s\",\"desc\":\"%s\"}",
            cmd->name,
            cmd->desc
            );
    }
    cogip::tracefd::out.printf("]}\n");
    cogip::tracefd::out.unlock();

    return EXIT_SUCCESS;
}

static int _exit_menu(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    if (current_menu->parent()) {
        cogip::tracefd::out.logf("Exit shell menu: %s", current_menu->name().c_str());
        current_menu->parent()->enter();
    }

    return EXIT_SUCCESS;
}

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

// command methods
command::command(const ::std::string &name, const ::std::string &desc, shell_command_handler_t handler)
    : name_(name), desc_(desc), handler_(handler)
{
    all_commands.insert(this);
}

command::~command()
{
    all_commands.erase(this);
}

// menu methods
menu::menu(
    const std::string &name, const std::string &cmd,
    menu *parent, func_cb_t enter_cb) : name_(name), cmd_(cmd), parent_(parent)
{
    enter_cb_ = enter_cb;

    if (cmd != "") {
        assert(all_menus.count(cmd) == 0);
        all_menus[cmd] = this;
    }

    std::string menu_desc = "Enter " + name;
    if (parent) {
        parent->push_back(new command(cmd, menu_desc, _cmd_enter_sub_menu));
    }
}

void menu::enter(void) const
{
    const menu *previous_menu = current_menu;
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

// Global functions
void start(void)
{
    char line_buf[SHELL_DEFAULT_BUFSIZE];

    // Add default global commands
    add_global_command(new command("_help_json", "Display available commands in JSON format", _display_json_help));
    add_global_command(new command("exit", "Exit current menu", _exit_menu));

    // Enter the root menu
    root_menu.enter();

    shell_run(current_commands, line_buf, SHELL_DEFAULT_BUFSIZE);
}

void add_global_command(command *command)
{
    global_commands.push_back(command);
    all_commands.insert(command);

    // Reload current menu
    if (current_menu) {
        current_menu->enter();
    }
}

void rename_command(const std::string &old_name, const std::string &new_name)
{
    for (auto cmd: all_commands) {
        if (cmd->name() == old_name) {
            cmd->set_name(new_name);
        }
    }

    // Reload menu
    current_menu->enter();
}

} // namespace shell

} // namespace cogip
