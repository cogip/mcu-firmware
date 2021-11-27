#include "shell_menu/shell_menu.hpp"
#include "shell_menu_private.hpp"

// Project includes
#include "tracefd/tracefd.hpp"

namespace cogip {

namespace shell {

Menu root_menu("Main", "");
std::map<std::string, Menu *> all_menus;
std::set<Command *> all_commands;
std::list<Command *> global_commands;
Menu *current_menu = nullptr;
shell_command_t current_commands[NB_SHELL_COMMANDS];
cogip::uartpb::UartProtobuf *uart_protobuf = nullptr;

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

void start(void)
{
    char line_buf[SHELL_DEFAULT_BUFSIZE];

    // Add default global commands
    add_global_command(new Command("_help_json", "Display available commands in JSON format", _display_json_help));
    add_global_command(new Command("exit", "Exit current menu", _exit_menu));

    // Enter the root menu
    root_menu.enter();

    shell_run(current_commands, line_buf, SHELL_DEFAULT_BUFSIZE);
}

void add_global_command(Command *command)
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

void register_uartpb(cogip::uartpb::UartProtobuf *uartpb_ptr)
{
    uart_protobuf = uartpb_ptr;
}

} // namespace shell

} // namespace cogip
