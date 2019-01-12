#ifndef UTILS_H_
#define UTILS_H_

typedef void (*func_cb_t)(void);

#define FALSE   (0)
#define TRUE    (!FALSE)

#define MIN(a, b)   (((a) < (b)) ? (a) : (b))
#define MAX(a, b)   (((a) > (b)) ? (a) : (b))

#include <stdio.h>
#define cons_printf printf
#define cons_scanf scanf
#define cons_getchar getchar

/* set interval to 20 milli-second */
#define THREAD_PERIOD_INTERVAL (20U * US_PER_MS)

#endif /* UTILS_H_ */
