#pragma once

typedef void (*func_cb_t)(void);

/**
 * @brief Compare two floating-point numbers (float) with a specified tolerance.
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
bool areDoublesEqual(float a, float b, float epsilon = 1e-3);

#define FALSE   (0)
#define TRUE    (!FALSE)

#define MIN(a, b)   (((a) < (b)) ? (a) : (b))
#define MAX(a, b)   (((a) > (b)) ? (a) : (b))

