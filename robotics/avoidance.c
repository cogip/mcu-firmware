#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "avoidance.h"
#include "obstacle.h"
#include "utils.h"

/* Obstacle list. Each obstacle is a polygon */
static polygon_t polygons[POLY_MAX];
/* Number of polygons */
static int nb_polygons = 0;
/* Number of dynamic polygons */
static int nb_dyn_polygons = 0;

/* List of visible points */
static pose_t valid_points[MAX_POINTS];
static uint8_t valid_points_count = 0;

static uint64_t graph[GRAPH_MAX_VERTICES];

static pose_t start_position = { .x = 0, .y = 0 };
static pose_t finish_position = { .x = 0, .y = 0 };

static polygon_t borders = { .count = 0, };

pose_t avoidance(uint8_t index)
{
    /* Build path graph */
    return dijkstra(1, index);
}

int update_graph(const pose_t *s, const pose_t *f)
{
    start_position = *s;
    finish_position = *f;

    /* Init all obstacles */
    if (nb_polygons == 0) {
        mach_fixed_obstacles_init();
    }

    if (borders.count == 0) {
        borders_init(&borders);
    }

    if (!is_point_in_polygon(&borders, finish_position)) {
        goto update_graph_error_finish_position;
    }

    /* Check that start and destination point are not in a polygon */
    for (int i = 0; i < (nb_polygons + nb_dyn_polygons); i++) {
        if (is_point_in_polygon(&polygons[i], finish_position)) {
            goto update_graph_error_finish_position;
        }
        if (is_point_in_polygon(&polygons[i], start_position)) {
            // TODO: Add return code
            //goto update_graph_error_start_position;
            // find nearest polygon point
            double min = DIJKSTRA_MAX_DISTANCE;
            pose_t *pose_tmp = &start_position;
            for (int j = 0; j < polygons[i].count; j++) {
                if (!is_point_in_polygon(&borders, polygons[i].points[j])) {
                    continue;
                }

                double distance = distance_points(&start_position, &polygons[i].points[j]);
                if (distance < min) {
                    min = distance;
                    pose_tmp = &polygons[i].points[j];
                }
            }

            start_position = *pose_tmp;
        }
    }

    valid_points_count = 0;
    valid_points[valid_points_count++] = start_position;
    valid_points[valid_points_count++] = finish_position;

    build_avoidance_graph();

    return 0;

update_graph_error_finish_position:
    return -1;
}

double distance_points(pose_t *a, pose_t *b)
{
    return sqrt((b->x - a->x) * (b->x - a->x)
                + (b->y - a->y) * (b->y - a->y));
}

/* Add a polygon to obstacle list */
int add_polygon(polygon_t *polygon)
{
    if (nb_polygons < POLY_MAX) {
        polygons[nb_polygons++] = *polygon;
        return 0;
    }
    else {
        return -1;
    }
}

/* Add a dynamic polygon to obstacle list */
int add_dyn_polygon(polygon_t *polygon)
{
    if ((nb_polygons + nb_dyn_polygons) < POLY_MAX) {
        polygons[nb_polygons + nb_dyn_polygons] = *polygon;
        nb_dyn_polygons++;
        return 0;
    }
    else {
        return -1;
    }
}

void reset_dyn_polygons(void)
{
    nb_dyn_polygons = 0;
}

/* Build obstacle graph
 * Each obstacle is a polygon.
 * List all  visible points : all points not contained in a polygon */
void build_avoidance_graph(void)
{
    /* For each polygon */
    for (int i = 0; i < (nb_polygons + nb_dyn_polygons); i++) {
        /* and for each vertice of that polygon */
        for (int p = 0; p < polygons[i].count; p++) {
            uint8_t collide = FALSE;
            /* Check if point is inside borders */
            if (!is_point_in_polygon(&borders, polygons[i].points[p])) {
                continue;
            }

            /* Check if this vertice is not inside an other polygon */
            for (int j = 0; j < (nb_polygons + nb_dyn_polygons); j++) {
                if (i == j) {
                    continue;
                }
                if (is_point_in_polygon(&polygons[j], polygons[i].points[p])) {
                    collide = TRUE;
                    break;
                }
            }
            /* If that point is not in an other polygon, add it to the list of valid points */
            if (!collide) {
                valid_points[valid_points_count++] = polygons[i].points[p];
            }
        }
    }

    /* For each segment of the valid points list */
    for (int p = 0; p < valid_points_count; p++) {
        for (int p2 = p + 1; p2 < valid_points_count; p2++) {
            uint8_t collide = FALSE;
            if (p != p2) {
                /* Check if that segment crosses a polygon */
                for (int i = 0; i < (nb_polygons + nb_dyn_polygons); i++) {
                    for (int v = 0; v < polygons[i].count; v++) {
                        pose_t p_next = ((v + 1 == polygons[i].count) ? polygons[i].points[0] : polygons[i].points[v + 1]);

                        if (is_segment_crossing_segment(valid_points[p], valid_points[p2], polygons[i].points[v], p_next)) {
                            collide = TRUE;
                            break;
                        }
                        /* Special case of internal crossing of a polygon */
                        int8_t index = get_point_index_in_polygon(&polygons[i], valid_points[p]);
                        int8_t index2 = get_point_index_in_polygon(&polygons[i], valid_points[p2]);
                        if ((index == 0) && (index2 == (polygons[i].count - 1))) {
                            continue;
                        }
                        if ((index2 == 0) && (index == (polygons[i].count - 1))) {
                            continue;
                        }
                        if ((index >= 0) && (index2 >= 0) && (abs(index - index2) != 1)) {
                            collide = TRUE;
                            break;
                        }
                        if (is_point_on_segment(valid_points[p], valid_points[p2], polygons[i].points[v])) {
                            collide = TRUE;
                            break;
                        }
                    }
                    if (collide) {
                        break;
                    }
                }
                /* If no collision, both points of the segment are added to the graph with distance between them */
                if ((p < GRAPH_MAX_VERTICES) && (p2 < GRAPH_MAX_VERTICES)) {
                    if (!collide) {
                        graph[p] |= (1 << p2);
                        graph[p2] |= (1 << p);
                    }
                    else {
                        graph[p] &= ~(1 << p2);
                        graph[p2] &= ~(1 << p);
                    }
                }
            }
        }
    }
    /*for (int i = 0; i < 6; i++)
       {
        printf("graph[%d] = %llx\n", i, graph[i]);
       }*/
}

uint8_t is_point_on_segment(pose_t a, pose_t b, pose_t o)
{
    uint8_t res = FALSE;

    if ((b.x - a.x) / (b.y - a.y) == (b.x - o.x) / (b.y - o.y)) {
        if (a.x < b.x) {
            if ((o.x < b.x) && (o.x > a.x)) {
                res = TRUE;
            }
        }
        else {
            if ((o.x < a.x) && (o.x > b.x)) {
                res = TRUE;
            }
        }
    }

    return res;
}

uint8_t is_segment_crossing_line(pose_t a, pose_t b, pose_t o, pose_t p)
{
    vector_t ao, ap, ab;
    double det = 0;

    ab.x = b.x - a.x;
    ab.y = b.y - a.y;
    ap.x = p.x - a.x;
    ap.y = p.y - a.y;
    ao.x = o.x - a.x;
    ao.y = o.y - a.y;

    det = (ab.x * ap.y - ab.y * ap.x) * (ab.x * ao.y - ab.y * ao.x);

    return (det < 0);
}

uint8_t is_segment_crossing_segment(pose_t a, pose_t b, pose_t o, pose_t p)
{
    if (!is_segment_crossing_line(a, b, o, p)) {
        return FALSE;
    }
    if (!is_segment_crossing_line(o, p, a, b)) {
        return FALSE;
    }
    return TRUE;
}

int8_t get_point_index_in_polygon(const polygon_t *polygons, pose_t p)
{
    uint8_t i;

    for (i = 0; i < polygons->count; i++) {
        if ((polygons->points[i].x == p.x) && (polygons->points[i].y == p.y)) {
            return i;
        }
    }

    return -1;
}

uint8_t is_point_in_polygon(const polygon_t *polygon, pose_t p)
{
    uint8_t i;
    double d;
    pose_t a, b;
    vector_t ab, ap;

    for (i = 0; i < polygon->count; i++) {
        a = polygon->points[i];
        b = (i == (polygon->count - 1) ? polygon->points[0] : polygon->points[i + 1]);

        ab.x = b.x - a.x;
        ab.y = b.y - a.y;
        ap.x = p.x - a.x;
        ap.y = p.y - a.y;

        d = ab.x * ap.y - ab.y * ap.x;

        if (d <= 0) {
            return FALSE;
        }
    }

    return TRUE;
}

pose_t dijkstra(uint16_t target, uint16_t index)
{
    uint8_t checked[GRAPH_MAX_VERTICES];
    double distance[GRAPH_MAX_VERTICES];
    uint16_t v;
    int i;
    double weight;
    double min_distance;
    int parent[GRAPH_MAX_VERTICES];
    int child[GRAPH_MAX_VERTICES];
    /* TODO: start should be a parameter. More clean even if start is always index 0 in our case */
    int start = 0;

    for (int i = 0; i <= valid_points_count; i++) {
        checked[i] = FALSE;
        distance[i] = DIJKSTRA_MAX_DISTANCE;
        parent[i] = -1;
    }

    distance[start] = 0;
    v = start;
    if (graph[v] == 0) {
        goto dijkstra_error_no_destination;
    }

    while ((v != target) && (checked[v] == FALSE)) {
        min_distance = DIJKSTRA_MAX_DISTANCE;
        checked[v] = TRUE;
        for (i = 0; i < valid_points_count; i++) {
            if (graph[v] & (1 << i)) {
                weight = (valid_points[v].x - valid_points[i].x);
                weight *= (valid_points[v].x - valid_points[i].x);
                weight += (valid_points[v].y - valid_points[i].y)
                          * (valid_points[v].y - valid_points[i].y);
                weight = sqrt(weight);
                if ((weight >= 0) && (distance[i] > (distance[v] + weight))) {
                    distance[i] = distance[v] + weight;
                    parent[i] = v;
                }
            }
        }
        for (i = 1; i < valid_points_count; i++) {
            if ((checked[i] == FALSE) && (min_distance > distance[i])) {
                min_distance = distance[i];
                v = i;
            }
        }
    }

    /* Build reverse path (from start to finish) */
    i = 1;
    while (parent[i] >= 0) {
        child[parent[i]] = i;
        i = parent[i];
    }

    /* Find n child in graph */
    i = 0;
    v = 0;
    while ((i != 1) && (v < index)) {
        i = child[i];
        v++;
    }

    /* Return point */
    return valid_points[i];

dijkstra_error_no_destination:
    return start_position;
}
