#pragma once

typedef void (*func_cb_t)(void);

/**
 * @brief Compare two floating-point numbers (double) with a specified tolerance.
 *
 * This function checks if the absolute difference between two doubles is less than a given tolerance (epsilon),
 * which helps to address the imprecision of floating-point calculations.
 *
 * @param[in]   a       The first floating-point number to compare.
 * @param[in]   b       The second floating-point number to compare.
 * @param[in]   epsilon The tolerance used for comparison (default value: 1e-3).
 * @return true         If the absolute difference between a and b is less than epsilon.
 * @return false        Otherwise.
 */
bool areDoublesEqual(double a, double b, double epsilon = 1e-3);

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

/// @brief Saturate a value between min and max
/// @param x Value to saturate
/// @param min minimun interval
/// @param max maxium insterval
/// @return satured value
inline double saturate(double x, double min, double max)
{
    if (x < min) {
        return min;
    }
    else if (x > max) {
        return max;
    }
    else {
        return x;
    }
}
