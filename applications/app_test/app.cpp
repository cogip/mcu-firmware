/* Project includes */
#include "app.hpp"
#include "app_conf.hpp"
#include "platform.hpp"

#define ENABLE_DEBUG        (0)
#include "debug.h"

cogip::obstacles::Polygon *_borders = nullptr;

/* Init all known fixed obstacles on map */
static void _fixed_obstacles_init(void)
{}

/* Init table borders */
static void _borders_init(void)
{
    static const std::list<cogip::cogip_defs::Coords> border_points = {
        {
            AVOIDANCE_BORDER_X_MIN,
            AVOIDANCE_BORDER_Y_MIN
        },
        {
            AVOIDANCE_BORDER_X_MAX,
            AVOIDANCE_BORDER_Y_MIN
        },
        {
            AVOIDANCE_BORDER_X_MAX,
            AVOIDANCE_BORDER_Y_MAX
        },
        {
            AVOIDANCE_BORDER_X_MIN,
            AVOIDANCE_BORDER_Y_MAX
        }
    };

    _borders = new cogip::obstacles::Polygon(&border_points);
}

const cogip::obstacles::Polygon *app_get_borders(void) { return _borders; };

void app_init(void)
{
    /* Init quadpid controller */
    pf_init_quadpid_params(ctrl_quadpid_params);

    _fixed_obstacles_init();
    _borders_init();
}

void app_init_tasks(void)
{}
