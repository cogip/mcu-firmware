#ifndef TRIGONOMETRY_H_
#define TRIGONOMETRY_H_

#ifndef M_PI
# define M_PI           3.14159265358979323846  /* pi */
#endif

#define square(__x)	(__x * __x)

#define RAD2DEG(a) (a * 360.0 / (2.0*M_PI))
#define DEG2RAD(a) (a * (2.0*M_PI) / 360.0)

double limit_angle_rad(double O);
double limit_angle_deg(double O);

#endif /* TRIGONOMETRY_H_ */
