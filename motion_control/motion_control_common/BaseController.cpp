// RIOT includes
#include "log.h"

// Project includes
#include "motion_control_common/BaseController.hpp"
#include "motion_control_common/BaseMetaController.hpp"
#include "utils.hpp"

namespace cogip {

namespace motion_control {

bool BaseController::set_meta(BaseMetaController* meta)
{
    if ((meta) && (meta_)) {
        // Build controller identification (type + name if available)
        char controller_id[128];
        if (name_.empty()) {
            snprintf(controller_id, sizeof(controller_id), "%s", type_name());
        } else {
            snprintf(controller_id, sizeof(controller_id), "%s: %.*s", type_name(),
                     static_cast<int>(name_.size()), name_.data());
        }

        // Build new meta identification
        char new_meta_id[128];
        if (meta->name().empty()) {
            snprintf(new_meta_id, sizeof(new_meta_id), "%s", meta->type_name());
        } else {
            snprintf(new_meta_id, sizeof(new_meta_id), "%s: %.*s", meta->type_name(),
                     static_cast<int>(meta->name().size()), meta->name().data());
        }

        // Build current meta identification
        char current_meta_id[128];
        if (meta_->name().empty()) {
            snprintf(current_meta_id, sizeof(current_meta_id), "%s", meta_->type_name());
        } else {
            snprintf(current_meta_id, sizeof(current_meta_id), "%s: %.*s", meta_->type_name(),
                     static_cast<int>(meta_->name().size()), meta_->name().data());
        }

        LOG_ERROR("Error: Controller '%s' already associated to MetaController '%s', "
                  "cannot add to MetaController '%s'.\n",
                  controller_id, current_meta_id, new_meta_id);
        return false;
    }
    meta_ = meta;
    return true;
}

} // namespace motion_control

} // namespace cogip
