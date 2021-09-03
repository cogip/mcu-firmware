// RIOT includes
#include "riot/chrono.hpp"
#include "riot/thread.hpp"

// Project includes
#include "cogip_defs/cogip_defs.hpp"
#include "shell_menu/shell_menu.hpp"
#include "obstacles.hpp"
#include "tracefd/tracefd.hpp"

// Application includes
#include "common_defs.h"
#include "lidar_utils.hpp"
#include "lidar_obstacles.hpp"

#ifdef MODULE_SHMEM
#include "shmem.h"
#endif

#define LIDAR_MINIMUN_INTENSITY 1000

uint32_t cycle = 1;

pose_t robot_state = {
    .coords = cogip::cogip_defs::Coords(
        0.0,
        1000.0
    ),
    .O = 0.0
};

static bool trace_on = false;

// Periodic task
#define TASK_PERIOD_MS 100

static void _init_border_obstacles(void)
{
    cogip::obstacles::List *border_obstacles = new cogip::obstacles::List();
    border_obstacles->push_back(new cogip::obstacles::Rectangle({ 0, -5 }, 0, 3000, 10));
    border_obstacles->push_back(new cogip::obstacles::Rectangle({ 0, 2005 }, 0, 3000, 10));
    border_obstacles->push_back(new cogip::obstacles::Rectangle({ -1505, 1000 }, 0, 10, 2000));
    border_obstacles->push_back(new cogip::obstacles::Rectangle({ 1505, 1000 }, 0, 10, 2000));
}

static int _cmd_trace_on_off(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    if (trace_on) {
        cogip::tracefd::out.logf("Deactivate traces");
        cogip::shell::rename_command("_trace_off", "_trace_on");
        trace_on = false;
    }
    else {
        cogip::tracefd::out.logf("Activate traces");
        cogip::shell::rename_command("_trace_on", "_trace_off");
        trace_on = true;
    }
    return 0;
}

static void _print_state(void)
{
    cogip::tracefd::out.lock();

    cogip::tracefd::out.printf(
        "{"
        "\"mode\":0,"
        "\"pose_current\":{\"x\":%.3lf,\"y\":%.3lf,\"O\":%.3lf},"
        "\"pose_order\":{\"x\":%.3lf,\"y\":%.3lf,\"O\":%.3lf},"
        "\"cycle\":%" PRIu32,
        robot_state.coords.x(), robot_state.coords.y(), robot_state.O,
        robot_state.coords.x(), robot_state.coords.y(), robot_state.O,
        cycle
        );

    cogip::tracefd::out.printf(",\"obstacles\":");
    cogip::obstacles::print_all_json(cogip::tracefd::out);

    cogip::tracefd::out.printf("}\n");

    cogip::tracefd::out.unlock();
}

int main(void)
{
    cogip::tracefd::out.logf("== Lidar obstacle detection example ==");

    // Add print data command
    cogip::shell::root_menu.push_back(
        new cogip::shell::Command("_trace_on", "Activate/deactivate trace", _cmd_trace_on_off));
    cogip::shell::root_menu.push_back(
        new cogip::shell::Command("_lidar_data", "Print Lidar data", lidar_cmd_print_data));

#ifdef MODULE_SHMEM
    cogip::shell::root_menu.push_back(new cogip::shell::Command(SHMEM_SET_KEY_CMD));
#endif

    _init_border_obstacles();

    lidar_start(LIDAR_MAX_DISTANCE, LIDAR_MINIMUN_INTENSITY);

    obstacle_updater_start(&robot_state);

    riot::thread trace_thread([] {
        while (true) {
            if (trace_on) {
                _print_state();
            }
            cycle++;
            riot::this_thread::sleep_for(std::chrono::milliseconds(TASK_PERIOD_MS));
        }
    });

    // Start shell
    cogip::shell::start();

    return 0;
}
