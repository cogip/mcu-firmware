#pragma once

#include "app_samples.hpp"

#include <cstdint>
#include "etl/stack.h"

namespace cogip {

namespace app {

class Context {
public:
    Context();
    ~Context() {};

    uint16_t score = 0;
    etl::map<SampleColor, Samples, SAMPLE_COLOR_COUNT> samples_in_gallery;
    etl::map<SampleColor, Samples, SAMPLE_COLOR_COUNT> samples_in_camp[4];

    etl::stack<Sample *, 8> samples_in_robot;

private:
};

/// Return the context of current application.
Context & app_get_context(void);

} // namespace app

} // namespace cogip
