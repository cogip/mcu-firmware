#include "trigonometry.h"

inline double limit_angle_rad(double O)
{
	while (O > M_PI)
		O -= 2.0 * M_PI;

	while (O < -M_PI)
		O += 2.0 * M_PI;

	return O;
}

inline double limit_angle_deg(double O)
{
	while (O > 180)
		O -= 360;

	while (O < -180)
		O += 360;

	return O;
}
