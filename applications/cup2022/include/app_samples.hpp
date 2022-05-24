#pragma once

#include "obstacles/Circle.hpp"

#include "PB_Samples.hpp"

#include <list>
#include <map>

#define APP_SAMPLES_MAX_DETECTED 10
#define APP_SAMPLE_RADIUS 75

namespace cogip {

namespace app {

/// Enum for sample colors
enum class SampleColor {
    Blue = 13,
    Rock = 17,
    Green = 36,
    Red = 47
};

/// Enum to identify each sample
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

enum class SampleLocation {
    OnTable,
    InRobot,
    Dropped,
    Unknown
};

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

using SampleIds = std::list<SampleId>;
using SamplesMap = std::map<SampleId, Sample *>;
using Samples = std::list<Sample *>;
using DetectedSamples = std::list<DetectedSample *>;

SamplesMap &app_samples_get(bool opposite=false);
Sample *app_samples_get_one(SampleId id, bool opposite=false);

void app_samples_init();

const DetectedSamples & app_samples_detect(void);

void app_samples_process(const PB_Samples<APP_SAMPLES_MAX_DETECTED> &samples);

}; // namespace app

}; // namespace cogip
