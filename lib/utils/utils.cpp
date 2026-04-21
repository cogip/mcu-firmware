#include "utils.hpp"

#include <cmath>

bool areFloatsEqual(float a, float b, float epsilon)
{
    return std::fabs(a - b) < epsilon;
}
