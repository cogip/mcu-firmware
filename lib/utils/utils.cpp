#include "utils.hpp"

#include <cmath>

bool areDoublesEqual(float a, float b, float epsilon) {
    return std::fabs(a - b) < epsilon;
}
