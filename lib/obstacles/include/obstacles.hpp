/*
 * Copyright (C) 2021 COGIP Robotics association <cogip35@gmail.com>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    lib_obstacles Obstacles module
 * @ingroup     lib
 * @brief       Obstacles module
 *
 * @{
 * @file
 * @brief       Public API for obstacles module
 *
 * @author      Eric Courtois <eric.courtois@gmail.com>
 * @author      Gilles DOFFE <g.doffe@gmail.com>
 */

#pragma once

// Standard includes
#include <cstdint>
#include <list>

// RIOT includes
#include "native_sched.h"
#include "riot/mutex.hpp"

// Project includes
#include "cogip_defs/cogip_defs.hpp"
#include "tracefd/File.hpp"

#if 0 // Unused?
/**
 * @brief   Default AABB obstacles table identifier definition
 */
typedef unsigned int obstacles_aabb_id_t;

/**
 * @brief Return the obstacle circumscribed circle radius
 * @param[in]   obstacle      obstacle
 * @return                    radius
 */
double obstacles_compute_radius(const obstacle_t *obstacle);
#endif

namespace cogip {

namespace obstacles {

class obstacle {
public:
    /// @brief Constructor
    /// @param[in]   center     obstacle center
    /// @param[in]   radius     obstacle circumscribed circle radius
    /// @param[in]   angle      absolute angle
    obstacle(coords_t center, double radius, double angle);

    /// @brief Destructor
    virtual ~obstacle() {};

    /// @brief Return bounding box of an obstacle. This bounding box has nb_vertices
    ///        and has a radius of ((1 + radius_margin) * radius).
    /// @param[in]   nb_vertices     number of bounding box vertices
    /// @param[in]   radius_margin   radius margin
    /// @return                      bounding box polygon
    polygon_t bounding_box(const uint8_t nb_vertices, double radius_margin) const;

    /// @brief Check if the given point is inside the obstacle
    /// @param[in]   p               point to check
    /// @return                      true if point is inside, false otherwise
    virtual bool is_point_inside(const coords_t &p) const = 0;

    /// @brief Check if a segment defined by two points A,B is crossing an obstacle.
    /// @param[in]   a               point A
    /// @param[in]   b               point B
    /// @param[in]   obstacle        obstacle
    /// @return                      true if [AB] crosses obstacle, false otherwise
    virtual bool is_segment_crossing(const coords_t &a, const coords_t &b) const = 0;

    /// @brief Find the nearest point of obstacle perimeter from given point.
    /// @param[in]   p               point to check
    /// @return                      position of nearest point
    virtual coords_t nearest_point(const coords_t &p) const = 0;

    /// @brief Print obstacles in JSON format
    /// @param[out]   out            Trace file descriptor
    virtual void print_json(cogip::tracefd::File &out) const = 0;

    /// @brief Return obstacle center
    const coords_t &center() const { return center_; };

    /// @brief Return obstacle circumscribed circle radius
    double radius() const { return radius_; };

    /// @brief Return absolute angle
    double angle() const { return angle_; };

protected:
    coords_t center_;                //// obstacle center
    double radius_;                  //// obstacle circumscribed circle radius
    double angle_;                   //// absolute angle
};

class circle : public obstacle {
public:
    circle(coords_t center, double radius, double angle);

    bool is_point_inside(const coords_t &p) const;
    bool is_segment_crossing(const coords_t &a, const coords_t &b) const;
    coords_t nearest_point(const coords_t &p) const;
    void print_json(cogip::tracefd::File &out) const;

private:
    /// @brief Check if a line defined by two points A,B is crossing a circle.
    /// @param[in]   a           point A
    /// @param[in]   b           point B
    /// @return                  true if (AB) crosses circle, false otherwise
    bool is_line_crossing_circle(const coords_t &a, const coords_t &b) const;

    circle_t circle_;
};

class polygon : public obstacle {
public:
    polygon(const std::list<coords_t> *points = nullptr);

    bool is_point_inside(const coords_t &p) const;
    bool is_segment_crossing(const coords_t &a, const coords_t &b) const;
    coords_t nearest_point(const coords_t &p) const;
    void print_json(cogip::tracefd::File &out) const;

protected:
    polygon_t polygon_;
};

class rectangle : public polygon {
public:
    rectangle(coords_t center, double angle,
              double length_x, double length_y);

    void print_json(cogip::tracefd::File &out) const;

private:
    double length_x_;        /// length on X axis when angle = 0
    double length_y_;        /// length on Y axis when angle = 0
};

class list: public std::list<obstacle *> {
public:
    list(uint32_t default_circle_radius = 0,
         uint32_t default_rectangle_width = 0,
         uint32_t min_distance = 0,
         uint32_t max_distance = 0);
    ~list();

    uint32_t default_circle_radius() { return default_circle_radius_; };
    uint32_t default_rectangle_width() { return default_rectangle_width_; };
    uint32_t min_distance() { return min_distance_; };
    uint32_t max_distance() { return max_distance_; };
    void lock() { mutex_.lock(); };
    void unlock() { mutex_.unlock(); };
    void print_json(cogip::tracefd::File &out) const;
    void clear();

private:
    uint32_t default_circle_radius_;     /// obstacle default radius
    uint32_t default_rectangle_width_;   /// obstacle of rectangle type default width
    uint32_t min_distance_;              /// minimun distance from origin to create an obstacle
    uint32_t max_distance_;              /// maximum distance from origin to create an obstacle
    riot::mutex mutex_;                  /// mutex protecting file access
};

/// @brief Check if the given point is inside an obstacle
/// @param[in]   p           point to check
/// @param[in]   filter      obstacle to filter
/// @return                  true if point is inside, false otherwise
bool is_point_in_obstacles(const coords_t &p, const obstacle *filter);

/// @brief Print all obstacles from all lists
/// @param[in]   out         trace file descriptor
void print_all_json(cogip::tracefd::File &out);

} // namespace obstacles

} // namespace cogip

/** @} */
