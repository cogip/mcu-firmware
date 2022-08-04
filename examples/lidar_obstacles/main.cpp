// RIOT includes
#include <ztimer.h>
#include "riot/thread.hpp"

// Project includes
#include "shell_menu/shell_menu.hpp"
#include "obstacles/obstacles.hpp"
#include "obstacles/List.hpp"
#include "obstacles/Rectangle.hpp"
#include "utils.hpp"

// Application includes
#include "common_defs.h"
#include "lidar_utils.hpp"
#include "lidar_obstacles.hpp"

#ifdef MODULE_SHMEM
#include "shmem.h"
#endif

#define LIDAR_MINIMUN_INTENSITY 1000

uint32_t cycle = 1;

cogip::cogip_defs::Pose robot_state = { 0.0, 1000.0, 0.0 };

static bool trace_on = false;

// Periodic task
#define TASK_PERIOD_USEC    (100 * US_PER_MS)

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
        COGIP_DEBUG_COUT("Deactivate traces");
        cogip::shell::rename_command("_trace_off", "_trace_on");
        trace_on = false;
    }
    else {
        COGIP_DEBUG_COUT("Activate traces");
        cogip::shell::rename_command("_trace_on", "_trace_off");
        trace_on = true;
    }
    return 0;
}

static void _print_state(void)
{
    COGIP_DEBUG_COUT(
        "{"
            << "\"mode\":0,"
            << "\"pose_current\":{"
                << "\"O\":" << robot_state.O()
                << "\"x\":" << robot_state.x()
                << ",\"y\":" << robot_state.y()
            << "},"
            << "\"pose_order\":{"
                << "\"O\":" << robot_state.O()
                << "\"x\":" << robot_state.x()
                << ",\"y\":" << robot_state.y()
            << "},"
            << "\"cycle\":" << cycle << ","
    );

    COGIP_DEBUG_COUT(",\"obstacles\":");
    cogip::obstacles::print_all_json();

    COGIP_DEBUG_COUT("}\n");
}

int main(void)
{
    COGIP_DEBUG_COUT("== Lidar obstacle detection example ==");

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

    obstacle_updater_start(robot_state);

    riot::thread trace_thread([] {
        // Init loop iteration start time
        ztimer_now_t loop_start_time = ztimer_now(ZTIMER_USEC);

        while (true) {
            if (trace_on) {
                _print_state();
            }
            cycle++;

            // Wait thread period to end
            ztimer_periodic_wakeup(ZTIMER_USEC, &loop_start_time, TASK_PERIOD_USEC);
        }
    });

    // Start shell
    cogip::shell::start();

    return 0;
}
