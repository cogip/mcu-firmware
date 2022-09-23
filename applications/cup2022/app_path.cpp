#include <vector>

#include "path/Path.hpp"
#include "app.hpp"
#include "platform.hpp"

// TODO:
// - Add anti-blocking activation for each point,
// - For each actions: keep a status if its done (to avoid multiple puck capture
//    if delivery was bypassed for any reason)
// - Absolute coordinate recalibration on games frames (?)
static cogip::path::Poses poses = {
    {
        225, 1000-225, 0,
        MAX_SPEED_LINEAR,
        MAX_SPEED_ANGULAR,
    },
    {
        450+125+200+125+225, -1000+225, 0,
        MAX_SPEED_LINEAR,
        MAX_SPEED_ANGULAR,
    },
    {
        3000-225, -1000+225, 90,
        MAX_SPEED_LINEAR,
        MAX_SPEED_ANGULAR,
    },
    {
        3000-225, 50+225, 90,
        MAX_SPEED_LINEAR,
        MAX_SPEED_ANGULAR,
    },
    {
        3000-(450+125+200+125+225), 1000-225, 180,
        MAX_SPEED_LINEAR,
        MAX_SPEED_ANGULAR,
    },
    {
        225, 1000-225, 0,
        MAX_SPEED_LINEAR,
        MAX_SPEED_ANGULAR,
    },
};

cogip::path::Path &app_get_path(void)
{
    static cogip::path::Path path(poses);
    return path;
}
