#include "app_context.hpp"

#include "planners/Planner.hpp"

namespace cogip {

namespace app {

cogip::planners::Planner *app_planner_get(void);

void app_planner_reset(void);

} // namespace app

} // namespace cogip
