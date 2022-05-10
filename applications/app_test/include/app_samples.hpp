#pragma once

#include "PB_Samples.hpp"

#define APP_SAMPLES_MAX_DETECTED 10

void app_samples_request(void);

void app_samples_process(const PB_Samples<APP_SAMPLES_MAX_DETECTED> &samples);
