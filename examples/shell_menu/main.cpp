#include <cstdio>

#include "shell_menu/shell_menu.hpp"

bool trace_on = false;

static int cmd_global(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    puts("Execute global command");
    return 0;
}

static int cmd_trace_on_off(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    if (trace_on) {
        puts("Deactivate traces");
        cogip::shell::rename_command("trace_off", "trace_on");
        trace_on = false;
    }
    else {
        puts("Activate traces");
        cogip::shell::rename_command("trace_on", "trace_off");
        trace_on = true;
    }
    return 0;
}

static int cmd_1(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    puts("Execute command 1 in main menu");
    return 0;
}

static int cmd_2(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    puts("Execute command 2 in main menu");
    return 0;
}

static int cmd_1_1(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    puts("Execute command 1 in Module 1");
    return 0;
}

static int cmd_1_2(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    puts("Execute command 2 in Module 1");
    return 0;
}

static int cmd_2_1(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    puts("Execute command 1 in Module 2");
    return 0;
}

static int cmd_2_2(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    puts("Execute command 2 in Module 2");
    return 0;
}

static int cmd_2_1_sub(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    puts("Execute command in Module 2 sub-menu");
    return 0;
}

void module1_enter_callback(void)
{
    puts("Execute module1 enter callback");
}

void module1_init(void)
{
    cogip::shell::Menu *menu = new cogip::shell::Menu(
        "Module 1 menu", "mod1", &cogip::shell::root_menu, module1_enter_callback);
    menu->push_back(new cogip::shell::Command("cmd_1", "Module 1 command 1", cmd_1_1));
    menu->push_back(new cogip::shell::Command("cmd_2", "Module 1 command 2", cmd_1_2));
}

void module2_init(void)
{
    cogip::shell::Menu *menu = new cogip::shell::Menu(
        "Module 2 menu", "mod2", &cogip::shell::root_menu);
    menu->push_back(new cogip::shell::Command("cmd_1", "Module 2 command 1", cmd_2_1));
    menu->push_back(new cogip::shell::Command("cmd_2", "Module 2 command 2", cmd_2_2));

    // Add a sub menu in the module
    cogip::shell::Menu *sub_menu = new cogip::shell::Menu(
        "Module 2 sub-menu", "mod2_sub", menu);
    sub_menu->push_back(
        new cogip::shell::Command("cmd_1", "Module 2 sub-menu command", cmd_2_1_sub));
}

void app_init(void)
{
    cogip::shell::add_global_command(
        new cogip::shell::Command("global", "Global command", cmd_global));
    cogip::shell::add_global_command(
        new cogip::shell::Command("trace_on", "Activate/deactivate trace", cmd_trace_on_off));

    cogip::shell::root_menu.push_back(
        new cogip::shell::Command("cmd_1", "Command 1", cmd_1));
    cogip::shell::root_menu.push_back(
        new cogip::shell::Command("cmd_2", "Command 2", cmd_2));
}

int main(void)
{
    puts("\n== Shell menu example ==");

    app_init();
    module1_init();
    module2_init();

    // Start shell
    cogip::shell::start();

    return 0;
}
