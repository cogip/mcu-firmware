#include <stdio.h>
#include "avoidance.h"

/* Init all known fixed obstacles on map */
void mach_fixed_obstacles_init(void)
{
	polygon_t polygon;
	uint8_t nb_vertices;

	polygon.count = 0;
	nb_vertices = 4;
	if (nb_vertices < POLY_MAX_POINTS)
	{
		polygon.points[polygon.count++] = (pose_t){.x = 1000, .y = 600};
		polygon.points[polygon.count++] = (pose_t){.x = 1400, .y = 600};
		polygon.points[polygon.count++] = (pose_t){.x = 1400, .y = 1000};
		polygon.points[polygon.count++] = (pose_t){.x = 1000, .y = 1000};
		add_polygon(&polygon);
	}
}

