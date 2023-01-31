#include "motion_control_common/BaseController.hpp"

namespace cogip {

namespace motion_control {

bool BaseController::set_meta(BaseMetaController *meta) {
    if (meta_) {
        std::cout << "Error: Controller already associated to a MetaController." << std::endl;
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
