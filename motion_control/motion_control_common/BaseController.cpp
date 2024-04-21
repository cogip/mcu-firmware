// Project includes
#include "motion_control_common/BaseController.hpp"
#include "utils.hpp"

namespace cogip {

namespace motion_control {

bool BaseController::set_meta(BaseMetaController *meta) {
    if ((meta) && (meta_)) {
        COGIP_DEBUG_COUT("Error: Controller already associated to a MetaController.");
        return false;
    }
    meta_ = meta;
    return true;
}

bool BaseController::is_index_valid(size_t index) {
    if (index != nb_inputs()) {
        std::cerr << "Wrong number of inputs, " << index << " given, " << nb_inputs() << " expected." << std::endl;

       return false;
    }

    return true;
}

} // namespace motion_control

} // namespace cogip
