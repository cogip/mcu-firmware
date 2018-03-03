#include "pid.h"

void pid_setup(PID_t *pid, const double kp, const double ki, const double kd)
{
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
}

void pid_reset(PID_t *pid)
{
	pid->previous_error = 0;
	pid->ti = 0;
}

double pid_controller(PID_t *pid, const double error)
{
	double p, i, d;

	/* proportional */
	p = error * pid->kp;

	/* integral */
	pid->ti += error;	/* error sum */

	/* integral limitation */
	if (pid->ti > INTEGRAL_LIMIT)
		pid->ti = INTEGRAL_LIMIT;
	else if (pid->ti < -INTEGRAL_LIMIT)
		pid->ti = -INTEGRAL_LIMIT;

	i = pid->ti * pid->ki;

	/* derivative */
	d = error - pid->previous_error;
	d *= pid->kd;
	pid->previous_error = error;	/* backup the previous error */

	return p + i + d;
}
