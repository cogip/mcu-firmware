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

#ifdef MODULE_UARTPB
cogip::uartpb::UartProtobuf *uart_protobuf = nullptr;
#endif

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

#ifdef MODULE_UARTPB
void register_uartpb(cogip::uartpb::UartProtobuf *uartpb_ptr)
{
    uart_protobuf = uartpb_ptr;
}

/// Execute a shell command callback using arguments from Protobuf message.
/// Command name is in the 'cmd' attribute of the Protobuf message.
/// Arguments are in a space-separated string in 'desc' attribute of the Protobuf message.
static void run_pb_command_(Command *command, const Command::PB_Message &pb_command)
{
    // Computed number of arguments
    int argc = 0;
    // List of arguments null-separated
    char args[COMMAND_DESC_MAX_LENGTH];
    // Array of pointers to each argument
    char *argv[COMMAND_MAX_PB_ARGS];
    // First argument is the command
    argv[argc++] = (char *)pb_command.cmd();
    if (pb_command.get_desc().get_length() != 0) {
        // If there are arguments to pass to the command in the 'desc' attribute
        size_t i;
        // Copy first argument pointer to 'argv'
        argv[argc++] = args;
        for (i = 0; i < pb_command.get_desc().get_length(); i++) {
            if (i >= COMMAND_DESC_MAX_LENGTH || argc >= COMMAND_MAX_PB_ARGS) {
                printf("Skip command '%s %s': arguments too long\n", pb_command.cmd(), pb_command.desc());
                return;
            }
            char c = pb_command.desc()[i];
            // Copy each argument in 'args'
            args[i] = c;
            if (c == ' ') {
                // Insert a null character between each argument
                args[i] = '\0';
                // Copy next argument pointer to 'argv'
                argv[argc++] = args + i + 1;
            }
        }
        // Add null-character after last argument
        args[i] = '\0';
    }
    // Execute shell command callback
    command->handler()(argc, argv);
}


// Handle a Protobuf command message
void handle_pb_command(const Command::PB_Message &pb_command)
{
    if (cogip::shell::current_menu == nullptr) {
        cogip::tracefd::out.logf(
            "Warning: received PB command before current_menu is initialized: %s %s\n",
            pb_command.cmd(), pb_command.desc());
        return;
    }

    // Search the command in current menu command
    for (auto command: *cogip::shell::current_menu) {
        if (command->name() == pb_command.cmd()) {
            run_pb_command_(command, pb_command);
            return;
        }
    }

    // If command was not found in current menu,
    // search the command in global commands
    for (auto command: cogip::shell::global_commands) {
        if (command->name() == pb_command.cmd()) {
            run_pb_command_(command, pb_command);
            return;
        }
    }
}
#endif

} // namespace shell

} // namespace cogip
