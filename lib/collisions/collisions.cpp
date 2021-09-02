/* Standard includes */
#include <stdbool.h>
#include <stdio.h>
#include <math.h>

/* Project includes */
#include "collisions.hpp"

static bool _is_point_on_segment(const coords_t *a, const coords_t *b, const coords_t *c);

static int8_t _get_point_index_in_polygon(const polygon_t *polygon, const coords_t *p)
{
    for (uint8_t i = 0; i < polygon->count; i++) {
        if ((polygon->points[i].x == p->x) && (polygon->points[i].y == p->y)) {
            return i;
        }
    }
    return -1;
}

/**
 * @brief Compute the distance between two points A and B.
 *
 * @param[in]   a           point A
 * @param[in]   b           point B
 *
 * @return                  distance between A and B
 */
static double _distance_points(const coords_t *a, const coords_t *b)
{
    return sqrt((b->x - a->x) * (b->x - a->x)
                + (b->y - a->y) * (b->y - a->y));
}

bool collisions_is_point_in_circle(const circle_t *circle, const coords_t *p)
{
    double d = _distance_points(&circle->center, p);

    if (d * d > circle->radius * circle->radius) {
        return false;
    }
    else {
        return true;
    }
}

bool collisions_is_point_in_polygon(const polygon_t *polygon, const coords_t *p)
{
    for (uint8_t i = 0; i < polygon->count; i++) {
        coords_t a = polygon->points[i];
        coords_t b = (i == (polygon->count - 1) ? polygon->points[0] : polygon->points[i + 1]);
        vector_t ab, ap;

        ab.x = b.x - a.x;
        ab.y = b.y - a.y;
        ap.x = p->x - a.x;
        ap.y = p->y - a.y;

        if (ab.x * ap.y - ab.y * ap.x <= 0) {
            return false;
        }
    }
    return true;
}

/**
 * @brief Check if a segment defined by two points A,B is crossing line
 * defined by two other points C,D.
 *
 * @param[in]   a           point A
 * @param[in]   b           point B
 * @param[in]   c           point C
 * @param[in]   d           point D
 *
 * @return                  true if [AB] crosses (CD), false otherwise
 */
static bool _is_segment_crossing_line(const coords_t *a, const coords_t *b, const coords_t *c, const coords_t *d)
{
    vector_t ac, ad, ab;
    double det = 0;

    ab.x = b->x - a->x;
    ab.y = b->y - a->y;
    ad.x = d->x - a->x;
    ad.y = d->y - a->y;
    ac.x = c->x - a->x;
    ac.y = c->y - a->y;

    det = (ab.x * ad.y - ab.y * ad.x) * (ab.x * ac.y - ab.y * ac.x);

    return (det < 0);
}

/**
 * @brief Check if a segment defined by two points A,B is crossing segment
 * defined by two other points C,D.
 *
 * @param[in]   a           point A
 * @param[in]   b           point B
 * @param[in]   c           point C
 * @param[in]   d           point D
 *
 * @return                  true if [AB] crosses [CD], false otherwise
 */
static bool _is_segment_crossing_segment(const coords_t *a, const coords_t *b,
                                            const coords_t *c, const coords_t *d)
{
    if (!_is_segment_crossing_line(a, b, c, d)) {
        return false;
    }
    if (!_is_segment_crossing_line(c, d, a, b)) {
        return false;
    }
    return true;
}

bool collisions_is_segment_crossing_polygon(const coords_t *a, const coords_t *b,
                                            const polygon_t *polygon)
{
    /* Check if that segment crosses a polygon */
    for (int i = 0; i < polygon->count; i++) {
        coords_t p_next = ((i + 1 == polygon->count) ? polygon->points[0] : polygon->points[i + 1]);

        if (_is_segment_crossing_segment(a, b, &polygon->points[i], &p_next)) {
            return true;
        }

        /* If A and B are vertices of the polygon */
        int8_t index = _get_point_index_in_polygon(polygon, a);
        int8_t index2 = _get_point_index_in_polygon(polygon, b);
        /* Consecutive vertices: no collision */
        if (((index == 0) && (index2 == (polygon->count - 1)))
            || ((index2 == 0) && (index == (polygon->count - 1)))) {
            continue;
        }
        /* If not consecutive vertices: collision */
        if ((index >= 0) && (index2 >= 0) && (abs(index - index2) != 1)) {
            return true;
        }

        /* If polygon vertice is on segment [AB] */
        if (_is_point_on_segment(a, b, &polygon->points[i])) {
            return true;
        }
    }

    return false;
}

/**
 * @brief Check if a line defined by two points A,B is crossing a circle.
 *
 * @param[in]   a           point A
 * @param[in]   b           point B
 * @param[in]   circle      circle
 *
 * @return                  true if (AB) crosses circle, false otherwise
 */
static bool _is_line_crossing_circle(const coords_t *a, const coords_t *b,
                                        const circle_t *circle)
{
    const coords_t *c = &circle->center;

    vector_t vect_ab;

    vect_ab.x = b->x - a->x;
    vect_ab.y = b->y - a->y;

    vector_t vect_ac;
    vect_ac.x = c->x - a->x;
    vect_ac.y = c->y - a->y;

    /* Norm of vector V */
    double numerator = vect_ab.x * vect_ac.y - vect_ab.y * vect_ac.x;
    if (numerator < 0) {
        numerator = -numerator;
    }

    /* Norm of vector U */
    double denominator = sqrt(vect_ab.x * vect_ab.x + vect_ab.y * vect_ab.y);

    /* Norm of vector CI where I is the nearest point of the line */
    double ci = numerator / denominator;

    /* If CI norm is less or equal to the circle radius, point I is inside the
     * circle */
    if (ci < circle->radius) {
        return true;
    }
    else {
        return false;
    }
}

bool collisions_is_segment_crossing_circle(const coords_t *a, const coords_t *b,
                                           const circle_t *circle)
{
    const coords_t *c = &circle->center;

    if (!_is_line_crossing_circle(a, b, circle)) {
        return false;
    }

    if (collisions_is_point_in_circle(circle, a)) {
        return true;
    }
    if (collisions_is_point_in_circle(circle, b)) {
        return true;
    }

    vector_t vect_ab, vect_ac, vect_bc;
    vect_ab.x = b->x - a->x;
    vect_ab.y = b->y - a->y;
    vect_ac.x = c->x - a->x;
    vect_ac.y = c->y - a->y;
    vect_bc.x = c->x - b->x;
    vect_bc.y = c->y - b->y;

    double scal1 = vect_ab.x * vect_ac.x + vect_ab.y * vect_ac.y;
    double scal2 = (-vect_ab.x) * vect_bc.x + (-vect_ab.y) * vect_bc.y;
    if (scal1 >= 0 && scal2 >= 0) {
        return true;
    }

    return false;
}

/**
 * @brief Check if a point C is placed on a segment defined by two points A,B.
 *
 * @param[in]   a           point A
 * @param[in]   b           point B
 * @param[in]   c           point C
 *
 * @return                  true if C is on [AB], false otherwise
 */
static bool _is_point_on_segment(const coords_t *a, const coords_t *b, const coords_t *c)
{
    bool res = false;

    if ((b->x - a->x) / (b->y - a->y) == (b->x - c->x) / (b->y - c->y)) {
        if (a->x < b->x) {
            if ((c->x < b->x) && (c->x > a->x)) {
                res = true;
            }
        }
        else {
            if ((c->x < a->x) && (c->x > b->x)) {
                res = true;
            }
        }
    }
    return res;
}

coords_t collisions_find_nearest_point_in_polygon(const polygon_t *polygon,
                                                  const coords_t *p)
{
    double min = UINT32_MAX;
    const coords_t *tmp = p;

    for (int j = 0; j < polygon->count; j++) {
        double distance = _distance_points(p, &polygon->points[j]);
        if (distance < min) {
            min = distance;
            tmp = &polygon->points[j];
        }
    }

    return *tmp;
}

coords_t collisions_find_nearest_point_in_circle(const circle_t *circle,
                                                 const coords_t *p)
{
    vector_t vect = {
        .x = p->x - circle->center.x,
        .y = p->y - circle->center.y,
    };

    double vect_norm = sqrt(vect.x * vect.x + vect.y * vect.y);

    return (coords_t) {
               .x = circle->center.x + (vect.x / vect_norm) * circle->radius,
               .y = circle->center.y + (vect.y / vect_norm) * circle->radius,
    };
}
