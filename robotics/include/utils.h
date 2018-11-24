#ifndef UTILS_H_
#define UTILS_H_

typedef void (*func_cb_t)(void);

#define FALSE   (0)
#define TRUE    (!FALSE)

#define MIN(a, b)    (((a) < (b)) ? (a) : (b))
#define MAX(a, b)    (((a) > (b)) ? (a) : (b))

#include <stdio.h>
#define cons_printf printf
#if !defined(BOARD_NATIVE)
#define cons_scanf custom_scanf
int custom_scanf(const char *format, ...);
#else
#define cons_scanf scanf
#endif
#define cons_getchar getchar

/* set interval to 20 milli-second */
#define THREAD_PERIOD_INTERVAL (20U * US_PER_MS)


#endif /* UTILS_H_ */
