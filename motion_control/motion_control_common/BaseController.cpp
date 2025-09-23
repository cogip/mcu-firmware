// RIOT includes
#include "log.h"

// Project includes
#include "motion_control_common/BaseController.hpp"
#include "utils.hpp"

namespace cogip {

namespace motion_control {

bool BaseController::set_meta(BaseMetaController *meta) {
    if ((meta) && (meta_)) {
        LOG_ERROR("Error: Controller already associated to a MetaController.\n");
        return false;
    }
    meta_ = meta;
    return true;
}

} // namespace motion_control

} // namespace cogip
