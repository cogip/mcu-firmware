// System include
#include <cstddef>
#include <cstdlib>
#include <iostream>

static size_t allocated = 0;

void* operator new(size_t sz) {
    allocated += sz;
    std::cerr << "[WARNING][allocating " << sz << " bytes] total = " << allocated << "\n";
    return std::malloc(sz);
}

