#pragma once

#define ROBOT_WIDTH                         354                     /**< Robot width (mm) */
#define ROBOT_MARGIN                        (ROBOT_WIDTH / 2)       /**< Point the most far from robot center (mm) */
#define OBSTACLE_RADIUS                     400                     /**< Obscale width */
#define BEACON_SUPPORT_DIAMETER             70                      /**< Size of the beacon support (a cylinder of 70mm diameter to a cube of 100mm width) */
#define LIDAR_MIN_DISTANCE                  (ROBOT_MARGIN + 100)
#define LIDAR_MAX_DISTANCE                  900
