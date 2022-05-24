#pragma once

#include "app_samples.hpp"

#include <cstdint>
#include <map>
#include <stack>

namespace cogip {

namespace app {

class Context {
public:
    Context();
    ~Context() {};

    uint16_t score = 0;
    std::map<SampleColor, Samples> samples_in_gallery;
    std::map<SampleColor, Samples> samples_in_camp[4];

    std::stack<Sample *> samples_in_robot;

private:
};

/// Return the context of current application.
Context & app_get_context(void);

} // namespace app

} // namespace cogip
