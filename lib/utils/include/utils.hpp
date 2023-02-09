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

#ifdef DEBUG
    #define COGIP_DEBUG_COUT(x)    (std::cout << x << std::endl)
    #define COGIP_DEBUG_CERR(x)    (std::cout << x << std::endl)
    #undef DEBUG
#else
    #define COGIP_DEBUG_COUT(x)
    #define COGIP_DEBUG_CERR(x)
#endif
