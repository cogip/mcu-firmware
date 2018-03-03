#ifndef LOG_H
#define LOG_H

#include <stdint.h>

#define LOG_LEVEL_ERROR    0
#define LOG_LEVEL_WARNING  1
#define LOG_LEVEL_INFO     2
#define LOG_LEVEL_DEBUG    3

#ifdef CONFIG_ENABLE_LOGGING
void print_log(int level, const char *function, const char *format, ...);
#define _print_log(level, ...) print_log(level, __FUNCTION__, __VA_ARGS__)
#else
#define _print_log(level, ...) do {} while(0)
#endif

#define print_dbg(...) _print_log(LOG_LEVEL_DEBUG, __VA_ARGS__)
#define print_info(...) _print_log(LOG_LEVEL_INFO, __VA_ARGS__)
#define print_warn(...) _print_log(LOG_LEVEL_WARNING, __VA_ARGS__)
#define print_err(...) _print_log(LOG_LEVEL_ERROR, __VA_ARGS__)

/* FIXME: this is not generic code, but some usage of it... should be place
 * elsewhere.
 */
/* Encoders wheel speeds */
#define LOG_IDX_SPEED_L			0
#define LOG_IDX_SPEED_R			1
/* Engine pwm commands */
#define LOG_IDX_MOTOR_L			2
#define LOG_IDX_MOTOR_R			3
/* Pid polar_t variales */
#define LOG_IDX_ROBOT_SPEED_D		4
//#define LOG_IDX_ROBOT_SPEED_A		5
#define LOG_IDX_SPEED_ORDER_D		5/*6*/
//#define LOG_IDX_SPEED_ORDER_A		7

typedef enum {
	COL_INT16,
	COL_DOUBLE,
	COL_END = -1,
} datalog_col_t;

/* The logs are consuming time when sent on UART
 * - http://superuser.com/questions/515554/what-is-the-minimum-delay-between-two-consecutive-rs232-frames
 *
 * In 115200 8N1 configuration, a char tooks 86.8us to be sent
 * thus not too much columns here below:
 */
#define COL_MAX 10
#define LOGNAME_MAX 20

typedef struct {
	uint16_t line_cur;		/* current line "timestamp" */

	char log_name[LOGNAME_MAX];	/* to store CSV basename */

	uint8_t col_nb;
	struct {
		datalog_col_t	type;
		const char *	name;
		uint8_t		visible;
	} columns[COL_MAX];

	/* datas are the active row */
	union {
		int16_t as_int16;
		double as_double;
	} datas[COL_MAX];
} datalog_t;

/*
 * Initialize a table for storing vectors. Columns are created at runtime.
 * The variable argument list should finished by a COL_END entry.
 * For each column to register, at least 2 arguments are required:
 * 1. the column data type, and 2. the columns header string.
 *
 * Example to create a one column array of integers:
 *   log_vect_init(&d, NULL, COL_INT16, "Col1Header", COL_END);
 */
void log_vect_init(datalog_t *d, const char *log_name, ...);

/* Reset current logs data indexes to 0, and configure CSV filename and visibles
 * columns. Last args should match valid column index, ending with -1 */
void log_vect_reset(datalog_t *d, const char *log_name, ...);

/* Set value for a cell (ie. using column id) for the current line */
void log_vect_setvalue(datalog_t *d, uint8_t idx, void * value);

/* Print line and move timestamp to next line for further data storage */
/* On first call line displayed, the header is also printed */
void log_vect_display_line(datalog_t *d);

/* Same as before, and also print the footer */
void log_vect_display_last_line(datalog_t *d);

#endif /* LOG_H */
