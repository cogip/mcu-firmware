#pragma once

// RIOT includes
#include <timex.h>

// System includes
#include <iostream>

typedef void (*func_cb_t)(void);

#define FALSE   (0)
#define TRUE    (!FALSE)

#define MIN(a, b)   (((a) < (b)) ? (a) : (b))
#define MAX(a, b)   (((a) > (b)) ? (a) : (b))

#define COGIP_DEBUG_COUT(x)    (std::cout << x << std::endl)
#define COGIP_DEBUG_CERR(x)    (std::cout << x << std::endl)

/* set interval to 20 milli-second */
#define THREAD_PERIOD_INTERVAL (20U * US_PER_MS)
