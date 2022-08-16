#pragma once

#include "obstacles/Circle.hpp"
#include "uartpb/ReadBuffer.hpp"

#include "PB_Samples.hpp"

#include "etl/list.h"
#include "etl/map.h"

#define APP_SAMPLES_MAX_DETECTED 10
#define APP_SAMPLE_RADIUS 75

namespace cogip {

namespace app {

/// Enum for sample colors
constexpr auto SAMPLE_COLOR_START_LINE = __LINE__;
enum class SampleColor {
    Blue = 13,
    Rock = 17,
    Green = 36,
    Red = 47
};
constexpr auto SAMPLE_COLOR_COUNT = __LINE__ - SAMPLE_COLOR_START_LINE - 3;

/// Enum to identify each sample
constexpr auto SAMPLE_ID_START_LINE = __LINE__;
enum class SampleId {
    TableFixedBlue,
    TableFixedGreen,
    TableFixedRed,
    TableRandomBlue,
    TableRandomGreen,
    TableRandomRed,
    ShedRed,
    ShedBlue,
    OutSideGreen,
    RackSideBlue,
    RackSideGreen,
    RackSideRed,
    RackTopBlue,
    RackTopGreen,
    RackTopRed,
};
constexpr auto SAMPLE_ID_COUNT = __LINE__ - SAMPLE_ID_START_LINE - 3;

constexpr auto SAMPLE_LOCATION_START_LINE = __LINE__;
enum class SampleLocation {
    OnTable,
    InRobot,
    Dropped,
    Unknown
};
constexpr auto SAMPLE_LOCATION_COUNT = __LINE__ - SAMPLE_LOCATION_START_LINE - 3;

class SampleObstacle: public cogip::obstacles::Circle {
public:
    SampleObstacle(const cogip::cogip_defs::Coords &center);
};

class Sample {
public:
    Sample(
        SampleId id, SampleColor color, bool hidden, bool known,
        SampleLocation loc=SampleLocation::OnTable
    );

    ~Sample() {};

    SampleId id() { return id_; };

    SampleColor color() const { return color_; };
    void set_color(SampleColor color) { color_ = color; };

    bool hidden() const { return hidden_; };
    void set_hidden(bool hidden) { hidden_ = hidden; };

    bool known() const { return known_; };
    void set_known(bool known) {known_ = known; };

    const cogip_defs::Coords & coords() const { return coords_; };
    void set_coords(double x, double y);

    float rot_x() const { return rot_x_; };
    float rot_y() const { return rot_y_; };
    float rot_z() const { return rot_z_; };
    void set_rotation(float x, float y, float z) { rot_x_ = x; rot_y_ = y; rot_z_ = z; };

    SampleLocation loc() const { return loc_; };
    void set_loc(SampleLocation loc) { loc_ = loc; };

    obstacles::Circle *obstacle() { return obstacle_; };

private:
    SampleId id_;
    SampleColor color_;
    bool hidden_;
    bool known_;
    cogip_defs::Coords coords_;
    float rot_x_;
    float rot_y_;
    float rot_z_;
    SampleLocation loc_;
    obstacles::Circle *obstacle_;
};

struct DetectedSample {
    SampleColor color;
    double x;
    double y;
};

using SampleIds = etl::list<SampleId, SAMPLE_ID_COUNT>;
using SamplesMap = etl::map<SampleId, Sample *, SAMPLE_ID_COUNT>;
using Samples = etl::list<Sample *, SAMPLE_ID_COUNT * 2>;
using DetectedSamples = etl::list<DetectedSample *, APP_SAMPLES_MAX_DETECTED>;

SamplesMap &app_samples_get(bool opposite=false);
Sample *app_samples_get_one(SampleId id, bool opposite=false);

void app_samples_init();

const DetectedSamples & app_samples_detect(void);

void app_samples_process(cogip::uartpb::ReadBuffer & buffer);

}; // namespace app

}; // namespace cogip
