#ifndef PID_H_
#define PID_H_

#define INTEGRAL_LIMIT		500000

/**
 * \struct PID
 */
typedef struct {
	double kp; /*!< proportional gain */
	double ki; /*!< integral gain */
	double kd; /*!< derivative gain */
	double ti; /*!< error sum */
	double previous_error; /*!< previous sum */
} PID_t;

/**
 * \fn pid_setup
 * \brief PID setup
 * \param pid struct PID
 * \param kp proportional gain
 * \param ki integral gain
 * \param kd derivative gain
 */
void pid_setup(PID_t *pid, const double kp, const double ki, const double kd);

/**
 * \fn pid_reset
 * \brief PID reset pid parameters
 * \param pid struct PID
 */
void pid_reset(PID_t *pid);

/**
 * \fn pid_controller
 * \brief compute pid controller
 * \param pid
 * \param error
 * \return the variable that will be adjusted by the pid
 */
double pid_controller(PID_t *pid, const double error);

#endif /* PID_H_ */
