/* Standard includes */
#include <stdbool.h>
#include <stdio.h>
#include <math.h>

/* Project includes */
#include "collisions.h"

double collisions_distance_points(const pose_t *a, const pose_t *b)
{
    return sqrt((b->coords.x - a->coords.x) * (b->coords.x - a->coords.x)
                + (b->coords.y - a->coords.y) * (b->coords.y - a->coords.y));
}

bool collisions_is_point_in_circle(const circle_t *circle, const pose_t *p)
{
    double d = collisions_distance_points(&circle->center, p);

    if (d * d > circle->radius * circle->radius) {
        return false;
    }
    else {
        return true;
    }
}

bool collisions_is_point_in_polygon(const polygon_t *polygon, const pose_t *p)
{
    for (uint8_t i = 0; i < polygon->count; i++) {
        pose_t a = polygon->points[i];
        pose_t b = (i == (polygon->count - 1) ? polygon->points[0] : polygon->points[i + 1]);
        vector_t ab, ap;

        ab.x = b.coords.x - a.coords.x;
        ab.y = b.coords.y - a.coords.y;
        ap.x = p->coords.x - a.coords.x;
        ap.y = p->coords.y - a.coords.y;

        if (ab.x * ap.y - ab.y * ap.x <= 0) {
            return false;
        }
    }
    return true;
}

bool collisions_is_segment_crossing_line(const pose_t *a, const pose_t *b, const pose_t *c, const pose_t *d)
{
    vector_t ac, ad, ab;
    double det = 0;

    ab.x = b->coords.x - a->coords.x;
    ab.y = b->coords.y - a->coords.y;
    ad.x = d->coords.x - a->coords.x;
    ad.y = d->coords.y - a->coords.y;
    ac.x = c->coords.x - a->coords.x;
    ac.y = c->coords.y - a->coords.y;

    det = (ab.x * ad.y - ab.y * ad.x) * (ab.x * ac.y - ab.y * ac.x);

    return (det < 0);
}

bool collisions_is_segment_crossing_segment(const pose_t *a, const pose_t *b,
                                            const pose_t *c, const pose_t *d)
{
    if (!collisions_is_segment_crossing_line(a, b, c, d)) {
        return false;
    }
    if (!collisions_is_segment_crossing_line(c, d, a, b)) {
        return false;
    }
    return true;
}

bool collisions_is_line_crossing_circle(const pose_t *a, const pose_t *b,
                                        const circle_t *circle)
{
    const pose_t *c = &circle->center;

    vector_t vect_ab;

    vect_ab.x = b->coords.x - a->coords.x;
    vect_ab.y = b->coords.y - a->coords.y;

    vector_t vect_ac;
    vect_ac.x = c->coords.x - a->coords.x;
    vect_ac.y = c->coords.y - a->coords.y;

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

bool collisions_is_segment_crossing_circle(const pose_t *a, const pose_t *b,
                                           const circle_t *circle)
{
    const pose_t *c = &circle->center;

    if (!collisions_is_line_crossing_circle(a, b, circle)) {
        return false;
    }

    if (collisions_is_point_in_circle(circle, a)) {
        return true;
    }
    if (collisions_is_point_in_circle(circle, b)) {
        return true;
    }

    vector_t vect_ab, vect_ac, vect_bc;
    vect_ab.x = b->coords.x - a->coords.x;
    vect_ab.y = b->coords.y - a->coords.y;
    vect_ac.x = c->coords.x - a->coords.x;
    vect_ac.y = c->coords.y - a->coords.y;
    vect_bc.x = c->coords.x - b->coords.x;
    vect_bc.y = c->coords.y - b->coords.y;

    double scal1 = vect_ab.x * vect_ac.x + vect_ab.y * vect_ac.y;
    double scal2 = (-vect_ab.x) * vect_bc.x + (-vect_ab.y) * vect_bc.y;
    if (scal1 >= 0 && scal2 >= 0) {
        return true;
    }

    return false;
}

bool collisions_is_point_on_segment(const pose_t *a, const pose_t *b, const pose_t *c)
{
    bool res = false;

    if ((b->coords.x - a->coords.x) / (b->coords.y - a->coords.y) == (b->coords.x - c->coords.x) / (b->coords.y - c->coords.y)) {
        if (a->coords.x < b->coords.x) {
            if ((c->coords.x < b->coords.x) && (c->coords.x > a->coords.x)) {
                res = true;
            }
        }
        else {
            if ((c->coords.x < a->coords.x) && (c->coords.x > b->coords.x)) {
                res = true;
            }
        }
    }
    return res;
}

inline double collisions_compute_slope(const pose_t *a, const pose_t *b)
{
    return (b->coords.y - a->coords.y) / (b->coords.x - a->coords.x);
}

inline double collisions_compute_ordinate(double slope, const pose_t *b)
{
    return (b->coords.y - slope * b->coords.x);
}

pose_t collisions_find_nearest_point_in_polygon(const polygon_t *polygon,
                                                const pose_t *p)
{
    double min = UINT32_MAX;
    const pose_t *pose_tmp = p;

    for (int j = 0; j < polygon->count; j++) {
        double distance = collisions_distance_points(p, &polygon->points[j]);
        if (distance < min) {
            min = distance;
            pose_tmp = &polygon->points[j];
        }
    }

    return *pose_tmp;
}

pose_t collisions_find_nearest_point_in_circle(const circle_t *circle,
                                               const pose_t *p)
{
    double slope = collisions_compute_slope(&circle->center, p);

    return (pose_t) {
               .coords.x = circle->center.coords.x * cos(slope),
               .coords.y = circle->center.coords.y * sin(slope),
               .O = 0,
    };
}
