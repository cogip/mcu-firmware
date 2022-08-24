// Firmware includes
#include "platform.hpp"
#include "trigonometry.h"

#include "app_camp.hpp"
#include "app_samples.hpp"

// RIOT includes
#include <event.h>

// System includes
#include <cmath>

#include "etl/pool.h"

namespace cogip {

namespace app {

static DetectedSamples _detected_samples;

constexpr cogip::uartpb::uuid_t sample_request_uuid = 3781855956;
constexpr cogip::uartpb::uuid_t sample_response_uuid = 1538397045;

static etl::pool<SampleObstacle, SAMPLE_ID_COUNT * 2 + 1> obstacles_pool;

typedef struct {
    event_t super;
    PB_Samples<APP_SAMPLES_MAX_DETECTED> pb_message;
} sample_event_t;

static sample_event_t _sample_event;
static event_queue_t _sample_queue;

SampleObstacle::SampleObstacle(const cogip::cogip_defs::Coords &center):
        cogip::obstacles::Circle(center, APP_SAMPLE_RADIUS + ROBOT_MARGIN)
{
    enabled_ = false;
};

Sample::Sample(
    SampleId id, SampleColor color, bool hidden, bool known, SampleLocation loc):
        id_(id), color_(color), hidden_(hidden), known_(known), loc_(loc)
{
    coords_.set_x(0);
    coords_.set_y(0);
    obstacle_ = obstacles_pool.create(coords_);
};

void Sample::set_coords(double x, double y)
{
    coords_.set_x(x);
    coords_.set_y(y);
    obstacle_->set_center(coords_);
}

void app_samples_init(etl::map<CampColor, SamplesMap, 2> & samples)
{
    pf_get_uartpb().register_message_handler(sample_response_uuid, uartpb::message_handler_t::create<app_samples_process>());

    event_queue_init_detached(&_sample_queue);
    _sample_event.super.list_node.next = nullptr;

    static etl::pool<Sample, OBSTACLES_MAX_NUMBER> samples_pool;
    static cogip::obstacles::List obstacles;

    Sample *sample = nullptr;

    // Create samples for yellow side
    sample = samples_pool.create(SampleId::TableFixedBlue, SampleColor::Blue, true, true, SampleLocation::OnTable);
    sample->set_coords(600, 550);
    sample->obstacle()->enable(true);
    obstacles.push_back(sample->obstacle());
    samples[CampColor::Yellow][SampleId::TableFixedBlue] = sample;

    sample = samples_pool.create(SampleId::TableFixedGreen, SampleColor::Green, true, true, SampleLocation::OnTable);
    sample->set_coords(670, 675);
    sample->obstacle()->enable(true);
    obstacles.push_back(sample->obstacle());
    samples[CampColor::Yellow][SampleId::TableFixedGreen] = sample;

    sample = samples_pool.create(SampleId::TableFixedRed, SampleColor::Red, true, true, SampleLocation::OnTable);
    sample->set_coords(600, 795);
    sample->obstacle()->enable(true);
    obstacles.push_back(sample->obstacle());
    samples[CampColor::Yellow][SampleId::TableFixedRed] = sample;

    sample = samples_pool.create(SampleId::TableRandomBlue, SampleColor::Blue, false, false, SampleLocation::OnTable);
    obstacles.push_back(sample->obstacle());
    samples[CampColor::Yellow][SampleId::TableRandomBlue] = sample;

    sample = samples_pool.create(SampleId::TableRandomGreen, SampleColor::Green, false, false, SampleLocation::OnTable);
    obstacles.push_back(sample->obstacle());
    samples[CampColor::Yellow][SampleId::TableRandomGreen] = sample;

    sample = samples_pool.create(SampleId::TableRandomRed, SampleColor::Red, false, false, SampleLocation::OnTable);
    obstacles.push_back(sample->obstacle());
    samples[CampColor::Yellow][SampleId::TableRandomRed] = sample;

    sample = samples_pool.create(SampleId::TableRandomRed, SampleColor::Red, false, false, SampleLocation::OnTable);
    obstacles.push_back(sample->obstacle());
    samples[CampColor::Yellow][SampleId::TableRandomRed] = sample;

    sample = samples_pool.create(SampleId::RackTopBlue, SampleColor::Blue, true, false, SampleLocation::OnTable);
    obstacles.push_back(sample->obstacle());
    samples[CampColor::Yellow][SampleId::RackTopBlue] = sample;

    sample = samples_pool.create(SampleId::RackTopGreen, SampleColor::Green, true, false, SampleLocation::OnTable);
    obstacles.push_back(sample->obstacle());
    samples[CampColor::Yellow][SampleId::RackTopGreen] = sample;

    sample = samples_pool.create(SampleId::RackTopRed, SampleColor::Red, true, false, SampleLocation::OnTable);
    obstacles.push_back(sample->obstacle());
    samples[CampColor::Yellow][SampleId::RackTopRed] = sample;

    sample = samples_pool.create(SampleId::RackSideBlue, SampleColor::Blue, true, false, SampleLocation::OnTable);
    obstacles.push_back(sample->obstacle());
    samples[CampColor::Yellow][SampleId::RackSideBlue] = sample;

    sample = samples_pool.create(SampleId::RackSideGreen, SampleColor::Green, true, false, SampleLocation::OnTable);
    obstacles.push_back(sample->obstacle());
    samples[CampColor::Yellow][SampleId::RackSideGreen] = sample;

    sample = samples_pool.create(SampleId::RackSideRed, SampleColor::Red, true, false, SampleLocation::OnTable);
    obstacles.push_back(sample->obstacle());
    samples[CampColor::Yellow][SampleId::RackSideRed] = sample;

    sample = samples_pool.create(SampleId::OutSideGreen, SampleColor::Green, true, false, SampleLocation::OnTable);
    sample->set_coords(1500+64.95, 300);
    obstacles.push_back(sample->obstacle());
    samples[CampColor::Yellow][SampleId::OutSideGreen] = sample;

    sample = samples_pool.create(SampleId::ShedBlue, SampleColor::Blue, true, false, SampleLocation::OnTable);
    sample->set_coords(1500-120.6, 1688.4);
    obstacles.push_back(sample->obstacle());
    samples[CampColor::Yellow][SampleId::ShedBlue] = sample;

    sample = samples_pool.create(SampleId::ShedRed, SampleColor::Red, true, false, SampleLocation::OnTable);
    sample->set_coords(1500-311.6, 1879.4);
    obstacles.push_back(sample->obstacle());
    samples[CampColor::Yellow][SampleId::ShedRed] = sample;

    // Copy samples for purple side
    for (const auto & item : samples[CampColor::Yellow]) {
        SampleId id = item.first;
        sample = item.second;
        Sample *new_sample = samples_pool.create(id, sample->color(), sample->hidden(), sample->known(), sample->loc());
        new_sample->set_coords(-sample->coords().x(), sample->coords().y());
        new_sample->obstacle()->enable(sample->obstacle()->enabled());
        obstacles.push_back(new_sample->obstacle());
        samples[CampColor::Purple][id] = new_sample;
    }
}

SamplesMap &app_samples_get(bool opposite)
{
    static etl::map<CampColor, SamplesMap, 2> samples;
    static bool initialized = false;
    if (! initialized) {
        app_samples_init(samples);
        initialized = true;
    }
    return samples[app_camp_get_color(opposite)];
}

Sample *app_samples_get_one(SampleId id, bool opposite)
{
    return app_samples_get(opposite)[id];
}

const DetectedSamples & app_samples_detect(void)
{
    static etl::pool<DetectedSample, APP_SAMPLES_MAX_DETECTED> detected_samples_pool;

    event_queue_claim(&_sample_queue);
    cogip::uartpb::UartProtobuf &uartpb = pf_get_uartpb();
    uartpb.send_message(sample_request_uuid);
    sample_event_t *event = (sample_event_t *)event_wait(&_sample_queue);

    auto & samples = event->pb_message.get_samples();
    for (DetectedSample *sample: _detected_samples) {
        detected_samples_pool.release(sample);
    }
    _detected_samples.clear();
    for (uint32_t i = 0; i < samples.get_length(); i++) {
        auto & sample = samples[i];
        const cogip::cogip_defs::Pose robot_pose = ctrl_get_pose_current(pf_get_ctrl());
        SampleColor color = (SampleColor)sample.get_tag();
        double local_x = sample.get_x();
        double local_y = sample.get_y();
        local_y = local_y/75*100;
        local_y -= (220 + ROBOT_WIDTH/2);
        double robot_angle = robot_pose.O();
        double global_x =
            robot_pose.x()
            + local_x * sin(robot_angle*M_PI/180)
            - local_y * cos(robot_angle*M_PI/180);
        double global_y =
            robot_pose.y()
            - local_x * cos(robot_angle*M_PI/180)
            - local_y * sin(robot_angle*M_PI/180);

        _detected_samples.push_back(detected_samples_pool.create(color, global_x, global_y));
    }
    return _detected_samples;
}

void app_samples_process(cogip::uartpb::ReadBuffer & buffer)
{
    EmbeddedProto::Error error = _sample_event.pb_message.deserialize(buffer);
    if (error != EmbeddedProto::Error::NO_ERRORS) {
        std::cout << "Samples: Protobuf deserialization error: " << static_cast<int>(error) << std::endl;
        return;
    }

    event_post(&_sample_queue, (event_t *)&_sample_event);
}

}; // namespace app

}; // namespace cogip
