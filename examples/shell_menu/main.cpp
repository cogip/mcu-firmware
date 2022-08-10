#include <cstdio>

#include "shell_menu/shell_menu.hpp"

bool trace_on = false;

static int func_global(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    puts("Execute global command");
    return 0;
}

static int func_trace_on_off(int argc, char **argv)
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

static int func_1(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    puts("Execute command 1 in main menu");
    return 0;
}

static int func_2(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    puts("Execute command 2 in main menu");
    return 0;
}

static int func_1_1(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    puts("Execute command 1 in Module 1");
    return 0;
}

static int func_1_2(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    puts("Execute command 2 in Module 1");
    return 0;
}

static int func_2_1(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    puts("Execute command 1 in Module 2");
    return 0;
}

static int func_2_2(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    puts("Execute command 2 in Module 2");
    return 0;
}

static int func_2_1_sub(int argc, char **argv)
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

cogip::shell::Command cmd_global = { "global", "Global command", func_global };
cogip::shell::Command cmd_trace_on_off = { "trace_on", "Activate/deactivate trace", func_trace_on_off };
cogip::shell::Command cmd_1 = { "cmd_1", "Command 1", func_1 };
cogip::shell::Command cmd_2 = { "cmd_2", "Command 2", func_2 };

cogip::shell::Menu menu_mod1 = { "Module 1 menu", "mod1", &cogip::shell::root_menu(), module1_enter_callback };
cogip::shell::Command cmd_1_1 = { "cmd_1", "Module 1 command 1", func_1_1 };
cogip::shell::Command cmd_1_2 = { "cmd_2", "Module 1 command 2", func_1_2 };

cogip::shell::Menu menu_mod2 = { "Module 2 menu", "mod2", &cogip::shell::root_menu() };
cogip::shell::Command cmd_2_1 = { "cmd_1", "Module 2 command 1", func_2_1 };
cogip::shell::Command cmd_2_2 = { "cmd_2", "Module 2 command 2", func_2_2 };

cogip::shell::Menu menu_mod2_sub = { "Module 2 sub-menu", "mod2_sub", &menu_mod2 };
cogip::shell::Command cmd_2_1_sub = { "cmd_1", "Module 2 sub-menu command", func_2_1_sub };

void module1_init(void)
{
    menu_mod1.push_back(&cmd_1_1);
    menu_mod1.push_back(&cmd_1_2);
}

void module2_init(void)
{
    menu_mod2.push_back(&cmd_2_1);
    menu_mod2.push_back(&cmd_2_2);

    // Add a sub menu in the module
    menu_mod2_sub.push_back(&cmd_2_1_sub);
}

void app_init(void)
{
    cogip::shell::add_global_command(&cmd_global);
    cogip::shell::add_global_command(&cmd_trace_on_off);

    cogip::shell::root_menu().push_back(&cmd_1);
    cogip::shell::root_menu().push_back(&cmd_2);
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
