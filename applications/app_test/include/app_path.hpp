#pragma once

#include "path/Path.hpp"
#include "app_actions.hpp"

namespace cogip {

namespace app {

class Path : public path::Path {
public:
    Path(Actions *actions);
    virtual ~Path() {};

    void new_action();

    const cogip::path::Pose *current_pose() override;
    void reset_current_pose_index() override;
    size_t operator++(int) override { return 0; };
    size_t operator--(int) override { return 0; };
    double current_max_speed_linear() override;
    double current_max_speed_angular() override;
    void horizontal_mirror_all_poses() override;
    void unreachable() override;
    void next() override;

private:
    Actions *actions_;
    Action *current_action_;
    Pose *current_pose_;
};

/// Return the path of current application.
Path & app_get_path(void);

/// Reset path.
void app_reset_path(void);

} // namespace app

} // namespace cogip
