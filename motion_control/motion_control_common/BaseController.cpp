// Project includes
#include "motion_control_common/BaseController.hpp"
#include <iostream>
#include "utils.hpp"

namespace cogip {

namespace motion_control {

bool BaseController::set_meta(BaseMetaController *meta) {
    if ((meta) && (meta_)) {
        std::cerr << "Error: Controller already associated to a MetaController." << std::endl;
        return false;
    }
    meta_ = meta;
    return true;
}

} // namespace motion_control

} // namespace cogip
