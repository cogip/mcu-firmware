#include <vector>

#include "path/Path.hpp"
#include "platform.hpp"

// TODO:
// - Add anti-blocking activation for each point,
// - For each actions: keep a status if its done (to avoid multiple puck capture
//    if delivery was bypassed for any reason)
// - Absolute coordinate recalibration on games frames (?)
static std::vector<cogip::path::Pose> poses = {
    {
        -1200, 1000, 0,
        MAX_SPEED
    },
    {
        1200, 1000, 180,
        MAX_SPEED
    },
    {
        -1200, 1000, 0,
        MAX_SPEED
    },
};

cogip::path::Path robot_path(poses, true);
