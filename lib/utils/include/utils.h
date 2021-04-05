#pragma once

typedef void (*func_cb_t)(void);

#define FALSE   (0)
#define TRUE    (!FALSE)

#define MIN(a, b)   (((a) < (b)) ? (a) : (b))
#define MAX(a, b)   (((a) > (b)) ? (a) : (b))

#define STR(x)  #x

/* set interval to 20 milli-second */
#define THREAD_PERIOD_INTERVAL (20U * US_PER_MS)
