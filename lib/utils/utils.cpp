#include "utils.hpp"

#include <cmath>

bool areDoublesEqual(double a, double b, double epsilon) {
    return std::fabs(a - b) < epsilon;
}
