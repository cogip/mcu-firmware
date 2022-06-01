#include "planners/Planner.hpp"

// System includes
#include <cstdlib>

// RIOT includes
#include "thread.h"

#ifdef MODULE_SHELL_PLANNERS
#include "shell_planners.hpp"
#endif // MODULE_SHELL_PLANNERS

namespace cogip {

namespace planners {

// Thread stack
char planner_thread_stack[THREAD_STACKSIZE_LARGE];

Planner::Planner(ctrl_t *ctrl, path::Path &path)
    : ctrl_(ctrl), path_(path), started_(false), allow_change_path_pose_(true), thread_exit_(false)
{
#ifdef MODULE_SHELL_PLANNERS
    pln_shell_init(this);
#endif // MODULE_SHELL_PLANNERS
};

Planner::~Planner() {
    thread_exit_ = true;
};

void Planner::start()
{
    ctrl_set_mode(ctrl_, CTRL_MODE_RUNNING);
    started_ = true;
}

void Planner::stop()
{
    ctrl_set_mode(ctrl_, CTRL_MODE_STOP);
    started_ = false;
}

static void *start_task(void *arg)
{
    ((Planner *)arg)->task_planner();
    return EXIT_SUCCESS;
}

void Planner::start_thread()
{
    thread_create(
        planner_thread_stack,
        sizeof(planner_thread_stack),
        THREAD_PRIORITY_MAIN - 2,
        0,
        start_task,
        this,
        "planner"
        );
}

} // namespace planners

} // namespace cogip
