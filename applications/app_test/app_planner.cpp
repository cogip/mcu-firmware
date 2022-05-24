#include "app_context.hpp"

#include "planners/Planner.hpp"
#include "planners/astar/AstarPlanner.hpp"
#include "platform.hpp"
#include "app_path.hpp"

namespace cogip {

namespace app {

static cogip::planners::Planner *_planner = nullptr;

cogip::planners::Planner *app_planner_get(void)
{
    if (! _planner) {
        _planner = new cogip::planners::AstarPlanner(pf_get_ctrl(), cogip::app::app_get_path());
    }
    return (cogip::planners::Planner *)_planner;
}

void app_planner_reset(void)
{
    std::cout << "app_planner_reset: current thread: " << thread_get_active()->name << std::endl;
    if (_planner) {
        delete _planner;
        _planner = nullptr;
        cogip::app::app_reset_path();
    }
    app_planner_get();
}

} // namespace app

} // namespace cogip
