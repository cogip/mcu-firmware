#include "path/Pose.hpp"
#include "platform.hpp"
#include "uartpb_config.hpp"
#include "trigonometry.h"

#include "app_actions.hpp"
#include "app_arm.hpp"
#include "app_camp.hpp"
#include "app_context.hpp"
#include "app_obstacles.hpp"
#include "app_samples.hpp"

#include "riot/chrono.hpp"
#include "riot/thread.hpp"

#include <limits>
#include <vector>
#include <iostream>
#include <functional>

namespace cogip {

namespace app {

enum class Arm {
    Left,
    Center,
    Right
};

#define MAX_SAMPLES_IN_ROBOT 6
#define SQRT_2 1.414213562

static bool _score_sent = false;

static PB_OutputMessage _pb_score;

static Actions *_actions = nullptr;

static ActionStrategy _strategy = ActionStrategy::Game;

struct ActionComparator
{
    bool operator ()(const Action *action1, const Action *action2)
    {
        return action1->weight() < action2->weight();
    }
};

Arm choose_arm(const Sample *sample)
{
    CampColor camp = app_camp_get_color();
    Arm arm = Arm::Center;

    const cogip::cogip_defs::Pose robot_pose = ctrl_get_pose_current(pf_get_ctrl());
    if (sample->hidden()) {
        if (sample->coords().y() > robot_pose.y()) {
            arm = Arm::Right;
        }
        else {
            arm = Arm::Left;
        }
    }
    if (camp == CampColor::Purple) {
        arm = (arm == Arm::Right) ? Arm::Left : Arm::Right;
    }

    return arm;
}

double get_arm_shift(Arm arm)
{
    int invert = app_camp_get_color() == CampColor::Yellow ? 1 : -1;
    double shift;
    switch (arm) {
        case Arm::Right:
            shift = - invert * SIDE_ARM_SHIFT; break;
        case Arm::Left:
            shift = invert * SIDE_ARM_SHIFT; break;
        case Arm::Center:
        default:
            shift = 0; break;
    }
    return shift;
}

Action::Action(
    const std::string &name, uint8_t direct_score, uint8_t potential_score) :
        name_(name), direct_score_(direct_score), potential_score_(potential_score)
{
    poses_ = new Poses();
}

Action::~Action()
{
    for (Pose *pose: *poses_) {
        delete pose;
    }
    poses_->clear();
    delete poses_;
}

Actions::~Actions()
{
    for (Action *action: *this) {
        delete action;
    }
    clear();
}

Action *Actions::new_action(Action *recycle_action)
{
    // std::cout << "[Actions::new_action] size = " << size() << std::endl;
    if (current_) {
        // std::cout << "[Actions::new_action] current = " << current_->name() << std::endl;
        current_->after_action();
    }

    current_ = nullptr;

    if (size()) {
        sort(ActionComparator());
        for (auto *action: *this) {
            // std::cout << action->weight() << " " << action->name() << std::endl;
        }
        Action *canditate = back();
        if (canditate->weight() == 0) {
            // std::cout << "[Actions::new_action] no more eligible actions" << std::endl;
        }
        else {
            current_ = canditate;
            pop_back();
            current_->before_action();
            // std::cout << "[Actions::new_action] selected = " << current_->name() << std::endl;
        }
    }
    else {
        // std::cout << "[Actions::new_action] no more actions" << std::endl;
    }

    if (recycle_action) {
        // std::cout << "[Actions::new_action] recycle action " << recycle_action->name() << std::endl;
        push_back(recycle_action);
    }

    return current_;
}

class ApprovalAction: public Action {
public:
    ApprovalAction(): Action("Approval action") {
        Pose *pose = new Pose(
            app_camp_adapt_distance(1500 - ROBOT_CENTER_TO_FACE),
            1000 - ROBOT_CENTER_TO_SIDE,
            app_camp_adapt_angle(180),
            NORMAL_SPEED_LINEAR, NORMAL_SPEED_ANGULAR, false
        );
        poses_->push_back(pose);

        pose = new Pose(
            app_camp_adapt_distance(50),
            1200,
            app_camp_adapt_angle(180),
            NORMAL_SPEED_LINEAR, NORMAL_SPEED_ANGULAR, false
        );
        poses_->push_back(pose);

        pose = new Pose(
            app_camp_adapt_distance(50),
            675,
            app_camp_adapt_angle(0),
            NORMAL_SPEED_LINEAR, NORMAL_SPEED_ANGULAR, false
        );
        pose->set_before_pose(std::bind(&ApprovalAction::before_push, this));
        poses_->push_back(pose);

        pose = new Pose(
            app_camp_adapt_distance(1500 - APP_SAMPLE_RADIUS*2 - ROBOT_CENTER_TO_FACE - 30),
            675,
            app_camp_adapt_angle(0),
            NORMAL_SPEED_LINEAR, NORMAL_SPEED_ANGULAR, false
        );
        poses_->push_back(pose);

        // std::cout << "ApprovalAction::poses_ = " << *poses_ << std::endl;
        pose_it_ = poses_->begin();
    };
    ~ApprovalAction() {};

    void before_push() {
        //std::cout << "ApprovalAction::before_push" << std::endl;
        //app_samples_get_one(SampleId::TableFixedBlue)->obstacle()->enable(false);
        //app_samples_get_one(SampleId::TableFixedRed)->obstacle()->enable(false);
        //app_samples_get_one(SampleId::TableFixedGreen)->obstacle()->enable(false);
    };


    float weight() const override {
        return 1000000;
    };
};

class PlayInLoopAction: public Action {
public:
    PlayInLoopAction(): Action("Play in loop action") {
        Pose *pose = new Pose(
            app_camp_adapt_distance(-1200),
            1000,
            app_camp_adapt_angle(0),
            LOW_SPEED_LINEAR, LOW_SPEED_ANGULAR
        );
        poses_->push_back(pose);

        pose = new Pose(
            app_camp_adapt_distance(1200),
            1000,
            app_camp_adapt_angle(180),
            LOW_SPEED_LINEAR, LOW_SPEED_ANGULAR
        );
        pose->set_after_pose(std::bind(&PlayInLoopAction::loop, this));
        poses_->push_back(pose);
        pose_it_ = poses_->begin();
    };
    ~PlayInLoopAction() {};

    void loop() {
        pose_it_ = poses_->begin();
    };

    float weight() const override {
        return 1000000;
    };
};

class StartAction: public Action {
public:
    StartAction(): Action("start") {
        Pose *pose = new Pose(
            app_camp_adapt_distance(1100 + ROBOT_CENTER_TO_FACE + 10),
            1000 - ROBOT_CENTER_TO_SIDE,
            app_camp_adapt_angle(180),
            NORMAL_SPEED_LINEAR, NORMAL_SPEED_ANGULAR, false
        );
        poses_->push_back(pose);
        app_get_context().score = 7;
        pose_it_ = poses_->begin();
    };
    ~StartAction() {};

    float weight() const override {
        return std::numeric_limits<float>::max();
    };
};

class ExcavationEndAction: public Action {
public:
    ExcavationEndAction(): Action("end") {
        Pose *pose = new Pose ({ app_camp_adapt_distance(525), 1375, -90, MAX_SPEED_LINEAR, MAX_SPEED_ANGULAR });
        poses_->push_back(pose);
        pose_it_ = poses_->begin();
    };
    ~ExcavationEndAction() {};

    float weight() const override {
        return 1;
    };

    void before_action() override {
        // std::cout << "ExcavationEndAction::before_action" << std::endl;
        // Disable excavation site obstacles
        std::map<CampColor, FixedObstacle *> & obstacles = app_get_excavation_sites_obstacles();
        obstacles[app_camp_get_color()]->enable(false);
    };

    void after_action() override {
        // std::cout << "ExcavationEndAction::after_action" << std::endl;
        app_get_context().score += 20;
        _pb_score.set_score(app_get_context().score);
        pf_get_uartpb()->send_message(_pb_score);
    };
};

class StaticAction: public Action {
public:
    StaticAction(
        const std::string &name,
        const Poses & poses): Action(name) {
        poses_->assign(poses.begin(), poses.end());
        pose_it_ = poses_->begin();
    };
    ~StaticAction() {};

    float weight() const override {
        return 1;
    };

private:
};

class GetFixedSampleAction: public Action {
public:
    GetFixedSampleAction(
        SampleId id,
        SampleIds neighbors,
        float weight,
        bool opposite=false
    ) : Action("Get fixed sample"),
        default_weight_(weight),
        opposite_(opposite)
    {
        sample_ = app_samples_get_one(id, opposite);
        for (SampleId neighbor_id: neighbors) {
            neighbors_.push_back(app_samples_get_one(neighbor_id, opposite));
        }
        pose_it_ = poses_->end();
    };
    ~GetFixedSampleAction() {};

    float weight() const override {
        if (app_get_context().samples_in_robot.size() >= MAX_SAMPLES_IN_ROBOT) {
            return 0;
        }
        const cogip::cogip_defs::Pose robot_pose = ctrl_get_pose_current(pf_get_ctrl());
        double distance_to_sample = robot_pose.distance(sample_->coords());
        return default_weight_ - (distance_to_sample / 1000);
    };

    void before_action() override {
        // std::cout << "GetFixedSampleAction::before_action" << std::endl;
        Pose *pose;
        Arm arm = choose_arm(sample_);
        const cogip::cogip_defs::Pose robot_pose = ctrl_get_pose_current(pf_get_ctrl());

        double dist_x = APP_SAMPLE_RADIUS + ROBOT_CENTER_TO_FACE;
        dist_x += arm == Arm::Center ? CENTRAL_ARM_LENGTH_DOWN : SIDE_ARM_LENGTH_DOWN;
        dist_x = app_camp_adapt_distance(dist_x);
        double x = sample_->coords().x() + dist_x;
        double y = sample_->coords().y() + get_arm_shift(arm);

        // 1. Approach if robot is far from the sample (> 150mm)
        if (robot_pose.distance(cogip_defs::Coords(x, y)) > 150) {
            pose = new Pose(x + app_camp_adapt_distance(100), y, app_camp_adapt_angle(180), MAX_SPEED_LINEAR, MAX_SPEED_ANGULAR);
            poses_->push_back(pose);
        }

        // 2. Take
        pose = new Pose(x, y, app_camp_adapt_angle(180), MAX_SPEED_LINEAR, MAX_SPEED_ANGULAR, false);
        pose->set_before_pose(std::bind(&GetFixedSampleAction::before_take, this));
        pose->set_after_pose(std::bind(&GetFixedSampleAction::after_take, this));
        poses_->push_back(pose);

        // 3. Step back
        pose = new Pose(x + app_camp_adapt_distance(100), y, app_camp_adapt_angle(180), MAX_SPEED_LINEAR, MAX_SPEED_ANGULAR);
        poses_->push_back(pose);

        pose_it_ = poses_->begin();
        // std::cout << "GetFixedSampleAction::before_action::poses = " << poses_ << std::endl;
    };

    void before_take() {
        // std::cout << "GetFixedSampleAction::before_take" << std::endl;
        sample_->obstacle()->enable(false);
        for (Sample *neighbor: neighbors_) {
            if (neighbor->loc() == SampleLocation::OnTable) {
                neighbor->obstacle()->enable(false);
            }
        }
    };

    void after_take() {
        // std::cout << "GetFixedSampleAction::after_take" << std::endl;
        sample_->obstacle()->enable(false);
        // TODO: Take sample with side arm => not hidden
        sample_->set_loc(SampleLocation::InRobot);
        sample_->set_hidden(false);
        app_get_context().samples_in_robot.push(sample_);
    };

    void after_action() override {
        // std::cout << "GetFixedSampleAction::after_action" << std::endl;
        for (Sample *neighbor: neighbors_) {
            if (neighbor->loc() == SampleLocation::OnTable) {
                neighbor->obstacle()->enable(true);
            }
        }
    };

private:
    Sample *sample_;
    std::list<Sample *> neighbors_;
    float default_weight_;
    bool opposite_;
};

class DropInGalleryAction: public Action {
public:
    DropInGalleryAction(
        SampleColor color,
        float weight
    ) : Action("Drop in gallery"),
        color_(color),
        default_weight_(weight)
    {
        switch (color_) {
            case SampleColor::Red:
                x_ = 450; break;
            case SampleColor::Green:
                x_ = 690; break;
            case SampleColor::Blue:
                x_ = 930; break;
            case SampleColor::Rock:
                // Should not happen
                default_weight_ = 0;
        }
        Pose *pose;
        x_ = app_camp_adapt_distance(x_);
        double y = ROBOT_CENTER_TO_FACE + CENTRAL_ARM_LENGTH_FRONT + 85 - 30;

        // // 1. Approach
        // pose = new Pose(x_, y + 50, -90, MAX_SPEED_LINEAR, MAX_SPEED_ANGULAR);
        // poses_->push_back(pose);

        // 2. Drop
        pose = new Pose(x_, y, -90, MAX_SPEED_LINEAR, MAX_SPEED_ANGULAR, false);
        pose->set_after_pose(std::bind(&DropInGalleryAction::drop_sample, this));
        poses_->push_back(pose);

        // // 3. Step back
        // pose = new Pose(x_, y + 50, -90, MAX_SPEED_LINEAR, MAX_SPEED_ANGULAR);
        // poses_->push_back(pose);

        pose_it_ = poses_->begin();
    };
    ~DropInGalleryAction() {};

    float weight() const override {
        if (app_get_context().samples_in_robot.size() == 0) {
            return 0;
        }
        if (app_get_context().samples_in_gallery[color_].size() >= 2) {
            return 0;
        }
        if (app_get_context().samples_in_robot.top()->color() != color_) {
            return 0;
        }
        return default_weight_;
    };

    void drop_sample() {
        // std::cout << "DropInGalleryAction::drop_sample" << std::endl;

        // TODO: Drop sample with center arm
        // uint8_t pose_in_gallery = app_get_context().samples_in_gallery[color_].size();

        Sample *sample = app_get_context().samples_in_robot.top();
        sample->set_loc(SampleLocation::Dropped);
        app_get_context().samples_in_gallery[color_].push_back(sample);
        app_get_context().score += 3;
        if (sample->color() == color_) {
            app_get_context().score += 3;
        }
        app_get_context().samples_in_robot.pop();
    };

private:
    SampleColor color_;
    float default_weight_;
    double x_;
};

class DetectRandomSamplesAction: public Action {
public:
    DetectRandomSamplesAction(
        float weight,
        bool opposite=false
    ) : Action("Detect random samples"),
        default_weight_(weight),
        opposite_(opposite)
    {
        double x;
        double y = 1375;

        if (opposite) {
            x = app_camp_adapt_distance(-350 + ROBOT_CENTER_TO_FACE + 180);
        }
        else {
            x = app_camp_adapt_distance(700 + ROBOT_CENTER_TO_FACE + 180);
        }
        Pose *pose = new Pose(x, y, app_camp_adapt_angle(180), MAX_SPEED_LINEAR, MAX_SPEED_ANGULAR);

        pose->set_after_pose(std::bind(&DetectRandomSamplesAction::detect_samples, this));
        poses_->push_back(pose);

        pose_it_ = poses_->begin();
    };
    ~DetectRandomSamplesAction() {};

    float weight() const override {
        return default_weight_;
    };

    void detect_samples() {
        // std::cout << "DetectRandomSamplesAction::detect_samples" << std::endl;
        // std::cout << "DetectRandomSamplesAction::detect_samples: current thread: " << thread_get_active()->name << std::endl;

        const DetectedSamples & detected_samples = app_samples_detect();
        for (DetectedSample *detected_sample: detected_samples) {
            SampleId id;
            SampleIds neigbors;
            switch (detected_sample->color) {
                case SampleColor::Red:
                    id = SampleId::TableRandomRed;
                    neigbors = { SampleId::TableRandomGreen, SampleId::TableRandomBlue };
                    break;
                case SampleColor::Green:
                    id = SampleId::TableRandomGreen;
                    neigbors = { SampleId::TableRandomRed, SampleId::TableRandomBlue };
                    break;
                case SampleColor::Blue:
                    id = SampleId::TableRandomBlue;
                    neigbors = { SampleId::TableRandomGreen, SampleId::TableRandomRed };
                    break;
                default:
                    // std::cout << "Unknown detected sample color: " << static_cast<int>(detected_sample->color) << std::endl;
                    continue;
            }
            Sample *sample = app_samples_get_one(id, opposite_);
            sample->set_coords(detected_sample->x, detected_sample->y);
            sample->set_known(true);
            sample->obstacle()->enable(true);

            std::map<CampColor, FixedObstacle *> & obstacles = app_get_excavation_sites_obstacles();
            obstacles[app_camp_get_color(opposite_)]->enable(false);

            // Get fixed sample on table
            _actions->push_back(new GetFixedSampleAction(id, neigbors, 900));
        }
    };

private:
    float default_weight_;
    bool opposite_;
};

class TopDispenserAction: public Action {
public:
    TopDispenserAction(
        float weight,
        bool opposite=false
    ) : Action("Take samples in top dispenser"),
        default_weight_(weight),
        opposite_(opposite)
    {
        double x = 150 /*+ SIDE_ARM_SHIFT*/;
        x = opposite ? -x : x;
        x = app_camp_adapt_distance(x);
        double y = 102 - 22 + 20 + ROBOT_CENTER_TO_FACE + CENTRAL_ARM_LENGTH_FRONT;

        Pose *pose = new Pose(x, y, -90, MAX_SPEED_LINEAR, MAX_SPEED_ANGULAR, false);
        pose->set_after_pose(std::bind(&TopDispenserAction::take_sample, this, SampleId::RackTopBlue));
        poses_->push_back(pose);

        pose = new Pose(x, y - 15, -90, LOW_SPEED_LINEAR, LOW_SPEED_ANGULAR, false);
        pose->set_after_pose(std::bind(&TopDispenserAction::take_sample, this, SampleId::RackTopGreen));
        poses_->push_back(pose);

        pose = new Pose(x, y - 30, -90, LOW_SPEED_LINEAR, LOW_SPEED_ANGULAR, false);
        pose->set_after_pose(std::bind(&TopDispenserAction::take_sample, this, SampleId::RackTopRed));
        poses_->push_back(pose);

        pose_it_ = poses_->begin();
    };
    ~TopDispenserAction() {};

    float weight() const override {
        if ((MAX_SAMPLES_IN_ROBOT - app_get_context().samples_in_robot.size()) < 3) {
            return 0;
        }
        return default_weight_;
    };

    virtual void take_sample(SampleId id) {
        // std::cout << "TopDispenserAction::take_sample(" << as_number(id) << ")" << std::endl;

        // TODO: get sample
        bool success = true;
        if (success) {
            Sample *sample = app_samples_get_one(id, opposite_);
            sample->set_known(true);
            sample->set_loc(SampleLocation::InRobot);
            app_get_context().samples_in_robot.push(sample);
            app_get_context().score++;
        }
    };

protected:
    float default_weight_;
    bool opposite_;
};

class SideDispenserAction: public Action {
public:
    SideDispenserAction(
        float weight
    ) : Action("Take samples in side dispenser"),
        default_weight_(weight)
    {
        name_ = "Take samples in side dispenser";
        double x = 1500 - 102 + 22 + 20 - ROBOT_CENTER_TO_FACE - CENTRAL_ARM_LENGTH_FRONT;
        double y = 1250;

        Pose *pose = new Pose(app_camp_adapt_distance(x), y, app_camp_adapt_angle(0), MAX_SPEED_LINEAR, MAX_SPEED_ANGULAR, false);
        pose->set_after_pose(std::bind(&SideDispenserAction::take_sample, this, SampleId::RackSideBlue));
        poses_->push_back(pose);

        pose = new Pose(app_camp_adapt_distance(x + 15), y, app_camp_adapt_angle(0), LOW_SPEED_LINEAR, LOW_SPEED_ANGULAR, false);
        pose->set_after_pose(std::bind(&SideDispenserAction::take_sample, this, SampleId::RackSideGreen));
        poses_->push_back(pose);

        pose = new Pose(app_camp_adapt_distance(x + 30), y, app_camp_adapt_angle(0), LOW_SPEED_LINEAR, LOW_SPEED_ANGULAR, false);
        pose->set_after_pose(std::bind(&SideDispenserAction::take_sample, this, SampleId::RackSideRed));
        poses_->push_back(pose);

        pose_it_ = poses_->begin();
    };
    float weight() const override {
        if ((MAX_SAMPLES_IN_ROBOT - app_get_context().samples_in_robot.size()) < 3) {
            return 0;
        }
        return default_weight_;
    };

    virtual void take_sample(SampleId id) {
        // std::cout << "SideDispenserAction::take_sample(" << as_number(id) << ")" << std::endl;

        // TODO: get sample
        bool success = true;
        if (success) {
            Sample *sample = app_samples_get_one(id);
            sample->set_known(true);
            sample->set_loc(SampleLocation::InRobot);
            app_get_context().samples_in_robot.push(sample);
            app_get_context().score++;
        }
    };

protected:
    float default_weight_;
};

class StatuetteAction: public Action {
public:
    StatuetteAction(
        float weight,
        bool opposite=false
    ) : Action("Statuette"),
        default_weight_(weight),
        opposite_(opposite)
    {
        Pose *pose;
        Sample *sample;
        cogip_defs::Coords statuette_center_on_pedestal(1245, 1749);

        // Approch pedestal
        pose = new Pose(
            app_camp_adapt_distance(statuette_center_on_pedestal.x() - (CENTRAL_ARM_LENGTH_FRONT*2 + ROBOT_CENTER_TO_FACE)/SQRT_2),
            statuette_center_on_pedestal.y() - (CENTRAL_ARM_LENGTH_FRONT*2 + ROBOT_CENTER_TO_FACE)/SQRT_2,
            app_camp_adapt_angle(45),
            MAX_SPEED_LINEAR, MAX_SPEED_ANGULAR);
        poses_->push_back(pose);

        // Take statuette on pedestal
        pose = new Pose(
            app_camp_adapt_distance(statuette_center_on_pedestal.x() - (CENTRAL_ARM_LENGTH_FRONT + ROBOT_CENTER_TO_FACE)/SQRT_2),
            statuette_center_on_pedestal.y() - (CENTRAL_ARM_LENGTH_FRONT + ROBOT_CENTER_TO_FACE)/SQRT_2,
            app_camp_adapt_angle(45),
            LOW_SPEED_LINEAR, LOW_SPEED_ANGULAR, false);
        pose->set_after_pose(std::bind(&StatuetteAction::take_statuette, this));
        poses_->push_back(pose);

        // Approach cabinet to drop statuette
        pose = new Pose(
            app_camp_adapt_distance(1275),
            ROBOT_CENTER_TO_FACE + CENTRAL_ARM_LENGTH_FRONT - 91 + 50,
            -90,
            MAX_SPEED_LINEAR, MAX_SPEED_ANGULAR);
        drop_statuette_pose1_ = poses_->insert(poses_->end(), pose);

        // Drop statuette
        pose = new Pose(
            app_camp_adapt_distance(1275),
            ROBOT_CENTER_TO_FACE + CENTRAL_ARM_LENGTH_FRONT - 91,
            -90,
            LOW_SPEED_LINEAR, LOW_SPEED_ANGULAR, false);
        pose->set_after_pose(std::bind(&StatuetteAction::drop_statuette, this));
        drop_statuette_pose2_ = poses_->insert(poses_->end(), pose);

        // Take side sample while we are here
        sample = app_samples_get_one(SampleId::OutSideGreen);
        pose = new Pose(
            app_camp_adapt_distance(sample->coords().x() - ROBOT_MARGIN/*must be ROBOT_CENTER_TO_FACE*/ - CENTRAL_ARM_LENGTH_DOWN),
            sample->coords().y(),
            app_camp_adapt_angle(0),
            MAX_SPEED_LINEAR, MAX_SPEED_ANGULAR);
        pose->set_after_pose(std::bind(&StatuetteAction::take_side_sample, this));
        poses_->push_back(pose);

        // Approach cabinet to take replica
        pose = new Pose(
            app_camp_adapt_distance(1115.15),
            ROBOT_CENTER_TO_FACE + CENTRAL_ARM_LENGTH_FRONT - 44.5 + 50,
            -90,
            LOW_SPEED_LINEAR, LOW_SPEED_ANGULAR);
        poses_->push_back(pose);

        // Take replica
        pose = new Pose(
            app_camp_adapt_distance(1115.15),
            ROBOT_CENTER_TO_FACE + CENTRAL_ARM_LENGTH_FRONT - 44.5,
            -90,
            MAX_SPEED_LINEAR, MAX_SPEED_ANGULAR, false);
        pose->set_after_pose(std::bind(&StatuetteAction::take_replica, this));
        poses_->push_back(pose);

        // Approach pedestal to drop replica
        pose = new Pose(
            app_camp_adapt_distance(statuette_center_on_pedestal.x() - (CENTRAL_ARM_LENGTH_FRONT*2 + ROBOT_CENTER_TO_FACE)/SQRT_2),
            statuette_center_on_pedestal.y() - (CENTRAL_ARM_LENGTH_FRONT*2 + ROBOT_CENTER_TO_FACE)/SQRT_2,
            app_camp_adapt_angle(45),
            MAX_SPEED_LINEAR, MAX_SPEED_ANGULAR);
        drop_replica_pose1_ = poses_->insert(poses_->end(), pose);

        // Drop replica
        pose = new Pose(
            app_camp_adapt_distance(statuette_center_on_pedestal.x() - (CENTRAL_ARM_LENGTH_FRONT + ROBOT_CENTER_TO_FACE)/SQRT_2),
            statuette_center_on_pedestal.y() - (CENTRAL_ARM_LENGTH_FRONT + ROBOT_CENTER_TO_FACE)/SQRT_2,
            app_camp_adapt_angle(45),
            LOW_SPEED_LINEAR, LOW_SPEED_ANGULAR, false);
        pose->set_after_pose(std::bind(&StatuetteAction::drop_replica, this));
        drop_replica_pose2_ = poses_->insert(poses_->end(), pose);

        // Take shed samples while we are here
        //   - Approach red
        sample = app_samples_get_one(SampleId::ShedRed);
        pose = new Pose(
            app_camp_adapt_distance(sample->coords().x() - (CENTRAL_ARM_LENGTH_DOWN*2 + ROBOT_CENTER_TO_FACE)/SQRT_2),
            sample->coords().y() - (CENTRAL_ARM_LENGTH_DOWN*2 + ROBOT_CENTER_TO_FACE)/SQRT_2,
            app_camp_adapt_angle(45),
            MAX_SPEED_LINEAR, MAX_SPEED_ANGULAR);
        poses_->push_back(pose);

        //   - Take red
        sample = app_samples_get_one(SampleId::ShedRed);
        pose = new Pose(
            app_camp_adapt_distance(sample->coords().x() - (CENTRAL_ARM_LENGTH_DOWN + ROBOT_CENTER_TO_FACE)/SQRT_2),
            sample->coords().y() - (CENTRAL_ARM_LENGTH_DOWN + ROBOT_CENTER_TO_FACE)/SQRT_2,
            app_camp_adapt_angle(45),
            LOW_SPEED_LINEAR, LOW_SPEED_ANGULAR, false);
        pose->set_after_pose(std::bind(&StatuetteAction::take_shed_sample, this, sample));
        poses_->push_back(pose);

        //   - Step back red
        sample = app_samples_get_one(SampleId::ShedRed);
        pose = new Pose(
            app_camp_adapt_distance(sample->coords().x() - (CENTRAL_ARM_LENGTH_DOWN*2 + ROBOT_CENTER_TO_FACE)/SQRT_2),
            sample->coords().y() - (CENTRAL_ARM_LENGTH_DOWN*2 + ROBOT_CENTER_TO_FACE)/SQRT_2,
            app_camp_adapt_angle(45),
            MAX_SPEED_LINEAR, MAX_SPEED_ANGULAR);
        poses_->push_back(pose);

        //   - Approach blue
        sample = app_samples_get_one(SampleId::ShedBlue);
        pose = new Pose(
            app_camp_adapt_distance(sample->coords().x() - (CENTRAL_ARM_LENGTH_DOWN*2 + ROBOT_CENTER_TO_FACE)/SQRT_2),
            sample->coords().y() - (CENTRAL_ARM_LENGTH_DOWN*2 + ROBOT_CENTER_TO_FACE)/SQRT_2,
            app_camp_adapt_angle(45),
            MAX_SPEED_LINEAR, MAX_SPEED_ANGULAR);
        poses_->push_back(pose);

        //   - Take blue
        sample = app_samples_get_one(SampleId::ShedBlue);
        pose = new Pose(
            app_camp_adapt_distance(sample->coords().x() - (CENTRAL_ARM_LENGTH_DOWN + ROBOT_CENTER_TO_FACE)/SQRT_2),
            sample->coords().y() - (CENTRAL_ARM_LENGTH_DOWN + ROBOT_CENTER_TO_FACE)/SQRT_2,
            app_camp_adapt_angle(45),
            LOW_SPEED_LINEAR, LOW_SPEED_ANGULAR, false);
        pose->set_after_pose(std::bind(&StatuetteAction::take_shed_sample, this, sample));
        poses_->push_back(pose);

        //   - Step back blue
        sample = app_samples_get_one(SampleId::ShedBlue);
        pose = new Pose(
            app_camp_adapt_distance(sample->coords().x() - (CENTRAL_ARM_LENGTH_DOWN*2 + ROBOT_CENTER_TO_FACE)/SQRT_2),
            sample->coords().y() - (CENTRAL_ARM_LENGTH_DOWN*2 + ROBOT_CENTER_TO_FACE)/SQRT_2,
            app_camp_adapt_angle(45),
            MAX_SPEED_LINEAR, MAX_SPEED_ANGULAR);
        poses_->push_back(pose);

        pose_it_ = poses_->begin();
    };

    float weight() const override {
        if (app_get_context().samples_in_robot.size() >= MAX_SAMPLES_IN_ROBOT) {
            return 0;
        }
        return default_weight_;
    };

    void take_statuette() {
        // std::cout << "StatuetteAction::take_statuette" << std::endl;
        // TODO: get statuette
        bool success = true;
        if (success) {
            app_get_context().score += 5;
        }
        else {
            poses_->erase(drop_statuette_pose1_);
            poses_->erase(drop_statuette_pose2_);
        }
    };

    void drop_statuette() {
        // std::cout << "StatuetteAction::drop_statuette" << std::endl;
        // TODO: drop statuette
        app_get_context().score += 15;
    };

    void take_side_sample() {
        // std::cout << "StatuetteAction::take_side_sample" << std::endl;
        // TODO: get sample
        bool success = true;
        if (success) {
            Sample *sample = app_samples_get_one(SampleId::OutSideGreen);
            sample->set_loc(SampleLocation::InRobot);
            app_get_context().samples_in_robot.push(sample);
            app_get_context().score++;
        }
    };

    void take_replica() {
        // std::cout << "StatuetteAction::take_replica" << std::endl;
        // TODO: get statuette
        bool success = true;
        if (! success) {
            poses_->erase(drop_replica_pose1_);
            poses_->erase(drop_replica_pose2_);
        }
    };

    void drop_replica() {
        // std::cout << "StatuetteAction::drop_replica" << std::endl;
        // TODO: drop statuette
        app_get_context().score += 10;
    };

    void take_shed_sample(Sample *sample) {
        // std::cout << "StatuetteAction::take_shed_sample(" << as_number(sample->id()) << ")" << std::endl;
        // TODO: get sample
        bool success = true;
        if (success) {
            sample->set_loc(SampleLocation::InRobot);
            app_get_context().samples_in_robot.push(sample);
            app_get_context().score++;
        }
    };


private:
    float default_weight_;
    bool opposite_;
    Poses::iterator drop_statuette_pose1_, drop_statuette_pose2_;
    Poses::iterator drop_replica_pose1_, drop_replica_pose2_;
};

class StatuetteOnlyAction: public Action {
public:
    StatuetteOnlyAction(
        float weight
    ) : Action("Statuette only"),
        default_weight_(weight)
    {
        Pose *pose;
        cogip_defs::Coords statuette_center_on_pedestal(1245, 1749);

        // Approch pedestal
        pose = new Pose(
            app_camp_adapt_distance(statuette_center_on_pedestal.x() - (CENTRAL_ARM_LENGTH_FRONT*2 + ROBOT_CENTER_TO_FACE)/SQRT_2),
            statuette_center_on_pedestal.y() - (CENTRAL_ARM_LENGTH_FRONT*2 + ROBOT_CENTER_TO_FACE)/SQRT_2,
            app_camp_adapt_angle(45),
            NORMAL_SPEED_LINEAR, NORMAL_SPEED_ANGULAR);
        pose->set_after_pose(std::bind(&StatuetteOnlyAction::approach_statuette, this));
        poses_->push_back(pose);

        // Take statuette on pedestal
        pose = new Pose(
            app_camp_adapt_distance(statuette_center_on_pedestal.x() - (CENTRAL_ARM_LENGTH_FRONT + ROBOT_CENTER_TO_FACE - 40)/SQRT_2),
            statuette_center_on_pedestal.y() - (CENTRAL_ARM_LENGTH_FRONT + ROBOT_CENTER_TO_FACE - 40)/SQRT_2,
            app_camp_adapt_angle(45),
            LOW_SPEED_LINEAR, LOW_SPEED_ANGULAR, false);
        pose->set_after_pose(std::bind(&StatuetteOnlyAction::take_statuette, this));
        poses_->push_back(pose);

        // Step back pedestal
        pose = new Pose(
            app_camp_adapt_distance(statuette_center_on_pedestal.x() - (CENTRAL_ARM_LENGTH_FRONT*2 + ROBOT_CENTER_TO_FACE)/SQRT_2),
            statuette_center_on_pedestal.y() - (CENTRAL_ARM_LENGTH_FRONT*2 + ROBOT_CENTER_TO_FACE)/SQRT_2,
            app_camp_adapt_angle(45),
            NORMAL_SPEED_LINEAR, NORMAL_SPEED_ANGULAR);
        pose->set_after_pose(std::bind(&StatuetteOnlyAction::stepback_statuette, this));
        poses_->push_back(pose);

        // Approach cabinet to drop statuette
        pose = new Pose(
            app_camp_adapt_distance(1275),
            ROBOT_CENTER_TO_FACE + CENTRAL_ARM_LENGTH_FRONT - 91 + 50,
            -90,
            NORMAL_SPEED_LINEAR, LOW_SPEED_ANGULAR);
        drop_statuette_pose1_ = poses_->insert(poses_->end(), pose);

        // Drop statuette
        pose = new Pose(
            app_camp_adapt_distance(1275),
            ROBOT_CENTER_TO_FACE + CENTRAL_ARM_LENGTH_FRONT - 91,
            -90,
            LOW_SPEED_LINEAR, LOW_SPEED_ANGULAR, false);
        pose->set_after_pose(std::bind(&StatuetteOnlyAction::drop_statuette, this));
        drop_statuette_pose2_ = poses_->insert(poses_->end(), pose);

        // Approach cabinet to take replica
        pose = new Pose(
            app_camp_adapt_distance(1115.15),
            ROBOT_CENTER_TO_FACE + CENTRAL_ARM_LENGTH_FRONT - 44.5 + 50,
            -90,
            NORMAL_SPEED_LINEAR, NORMAL_SPEED_ANGULAR);
        pose->set_after_pose(std::bind(&StatuetteOnlyAction::approach_replica, this));
        poses_->push_back(pose);

        // Take replica
        pose = new Pose(
            app_camp_adapt_distance(1115.15),
            ROBOT_CENTER_TO_FACE + CENTRAL_ARM_LENGTH_FRONT - 44.5,
            -90,
            LOW_SPEED_LINEAR, LOW_SPEED_ANGULAR, false);
        pose->set_after_pose(std::bind(&StatuetteOnlyAction::take_replica, this));
        poses_->push_back(pose);

        // Step back cabinet
        pose = new Pose(
            app_camp_adapt_distance(1115.15),
            ROBOT_CENTER_TO_FACE + CENTRAL_ARM_LENGTH_FRONT - 44.5 + 50,
            -90,
            LOW_SPEED_LINEAR, LOW_SPEED_ANGULAR, true);
        pose->set_after_pose(std::bind(&StatuetteOnlyAction::approach_replica, this));
        poses_->push_back(pose);

        // Approach cabinet to drop replica
        pose = new Pose(
            app_camp_adapt_distance(1115.15),
            ROBOT_CENTER_TO_FACE + CENTRAL_ARM_LENGTH_FRONT - 44.5 + 50,
            -90,
            NORMAL_SPEED_LINEAR, LOW_SPEED_ANGULAR);
        pose->set_after_pose(std::bind(&StatuetteOnlyAction::stepback_replica, this));
        poses_->push_back(pose);

        // Approach pedestal to drop replica
        pose = new Pose(
            app_camp_adapt_distance(statuette_center_on_pedestal.x() - (CENTRAL_ARM_LENGTH_FRONT*2 + ROBOT_CENTER_TO_FACE - 40)/SQRT_2),
            statuette_center_on_pedestal.y() - (CENTRAL_ARM_LENGTH_FRONT*2 + ROBOT_CENTER_TO_FACE - 40)/SQRT_2,
            app_camp_adapt_angle(45),
            NORMAL_SPEED_LINEAR, LOW_SPEED_ANGULAR);
        drop_replica_pose1_ = poses_->insert(poses_->end(), pose);

        // Drop replica
        pose = new Pose(
            app_camp_adapt_distance(statuette_center_on_pedestal.x() - (CENTRAL_ARM_LENGTH_FRONT + ROBOT_CENTER_TO_FACE)/SQRT_2),
            statuette_center_on_pedestal.y() - (CENTRAL_ARM_LENGTH_FRONT + ROBOT_CENTER_TO_FACE)/SQRT_2,
            app_camp_adapt_angle(45),
            LOW_SPEED_LINEAR, LOW_SPEED_ANGULAR, false);
        pose->set_after_pose(std::bind(&StatuetteOnlyAction::drop_replica, this));
        drop_replica_pose2_ = poses_->insert(poses_->end(), pose);

        pose_it_ = poses_->begin();
    };

    float weight() const override {
        return default_weight_;
    };

    void approach_statuette() {
        // std::cout << "StatuetteOnlyAction::approach_statuette" << std::endl;
        app_central_arm_gripping_statuette();
    };

    void take_statuette() {
        // std::cout << "StatuetteOnlyAction::take_statuette" << std::endl;
        // TODO: get statuette
        bool success = true;
        if (success) {
            app_get_context().score += 5;
        }
        else {
            poses_->erase(drop_statuette_pose1_);
            poses_->erase(drop_statuette_pose2_);
        }
    };

    void stepback_statuette() {
        // std::cout << "StatuetteOnlyAction::stepback_statuette" << std::endl;
        app_central_arm_gripping_statuette_up();
    };

    void drop_statuette() {
        // std::cout << "StatuetteOnlyAction::drop_statuette" << std::endl;
        app_central_arm_releasing_statuette();
        app_get_context().score += 15;
    };

    void approach_replica() {
        // std::cout << "StatuetteOnlyAction::approach_replica" << std::endl;
        app_central_arm_gripping_replica();
    };

    void take_replica() {
        // std::cout << "StatuetteOnlyAction::take_replica" << std::endl;
        // TODO: get statuette
        bool success = true;
        if (! success) {
            poses_->erase(drop_replica_pose1_);
            poses_->erase(drop_replica_pose2_);
        }
    };

    void stepback_replica() {
        // std::cout << "StatuetteOnlyAction::stepback_replica" << std::endl;
        app_central_arm_gripping_replica_up();
    };

    void drop_replica() {
        // std::cout << "StatuetteOnlyAction::drop_replica" << std::endl;
        app_central_arm_releasing_replica();
        app_get_context().score += 10;
    };

private:
    float default_weight_;
    Poses::iterator drop_statuette_pose1_, drop_statuette_pose2_;
    Poses::iterator drop_replica_pose1_, drop_replica_pose2_;
};

class DropInShedAction: public Action {
public:
    DropInShedAction(
        float weight
    ) : Action("Drop in shed"),
        default_weight_(weight)
    {
        Pose *pose;
        double x, y;
        cogip_defs::Coords statuette_center_on_pedestal(1245, 1749);

        // Drop side sample
        x = statuette_center_on_pedestal.x();
        y = statuette_center_on_pedestal.y();
        x += (APP_SAMPLE_RADIUS + 5)/SQRT_2;
        y -= (APP_SAMPLE_RADIUS + 5)/SQRT_2;
        x -= (APP_SAMPLE_RADIUS + 5)/SQRT_2;
        y -= (APP_SAMPLE_RADIUS + 5)/SQRT_2;
        x -= (CENTRAL_ARM_LENGTH_DOWN + ROBOT_CENTER_TO_FACE + 5)/SQRT_2;
        y -= (CENTRAL_ARM_LENGTH_DOWN + ROBOT_CENTER_TO_FACE + 5)/SQRT_2;
        pose = new Pose(app_camp_adapt_distance(x), y, app_camp_adapt_angle(45), MAX_SPEED_LINEAR, MAX_SPEED_ANGULAR);
        pose->set_after_pose(std::bind(&DropInShedAction::drop_sample, this));
        poses_->push_back(pose);

        // Push side sample
        x = statuette_center_on_pedestal.x();
        y = statuette_center_on_pedestal.y();
        x += (APP_SAMPLE_RADIUS + 5)/SQRT_2;
        y -= (APP_SAMPLE_RADIUS + 5)/SQRT_2;
        x -= (ROBOT_CENTER_TO_FACE + 5)/SQRT_2;
        y -= (ROBOT_CENTER_TO_FACE + 5)/SQRT_2;
        pose = new Pose(app_camp_adapt_distance(x), y, app_camp_adapt_angle(45), LOW_SPEED_LINEAR, LOW_SPEED_ANGULAR, false);
        pose->set_before_pose(std::bind(&DropInShedAction::enable_shed_obstacle, this, false));
        poses_->push_back(pose);

        // Step back
        x = statuette_center_on_pedestal.x();
        y = statuette_center_on_pedestal.y();
        x += (APP_SAMPLE_RADIUS + 5)/SQRT_2;
        y -= (APP_SAMPLE_RADIUS + 5)/SQRT_2;
        x -= (APP_SAMPLE_RADIUS + 5)/SQRT_2;
        y -= (APP_SAMPLE_RADIUS + 5)/SQRT_2;
        x -= (CENTRAL_ARM_LENGTH_DOWN + ROBOT_CENTER_TO_FACE + 5)/SQRT_2;
        y -= (CENTRAL_ARM_LENGTH_DOWN + ROBOT_CENTER_TO_FACE + 5)/SQRT_2;
        pose = new Pose(app_camp_adapt_distance(x), y, app_camp_adapt_angle(45), MAX_SPEED_LINEAR, MAX_SPEED_ANGULAR);
        pose->set_after_pose(std::bind(&DropInShedAction::enable_shed_obstacle, this, true));
        poses_->push_back(pose);

        // Drop top sample
        x = statuette_center_on_pedestal.x();
        y = statuette_center_on_pedestal.y();
        x -= (APP_SAMPLE_RADIUS + 5)/SQRT_2;
        y += (APP_SAMPLE_RADIUS + 5)/SQRT_2;
        x -= (APP_SAMPLE_RADIUS + 5)/SQRT_2;
        y -= (APP_SAMPLE_RADIUS + 5)/SQRT_2;
        x -= (CENTRAL_ARM_LENGTH_DOWN + ROBOT_CENTER_TO_FACE + 5)/SQRT_2;
        y -= (CENTRAL_ARM_LENGTH_DOWN + ROBOT_CENTER_TO_FACE + 5)/SQRT_2;
        pose = new Pose(app_camp_adapt_distance(x), y, app_camp_adapt_angle(45), MAX_SPEED_LINEAR, MAX_SPEED_ANGULAR);
        pose->set_after_pose(std::bind(&DropInShedAction::drop_sample, this));
        poses_->push_back(pose);

        // Push top sample
        x = statuette_center_on_pedestal.x();
        y = statuette_center_on_pedestal.y();
        x -= (APP_SAMPLE_RADIUS + 5)/SQRT_2;
        y += (APP_SAMPLE_RADIUS + 5)/SQRT_2;
        x -= (ROBOT_CENTER_TO_FACE + 5)/SQRT_2;
        y -= (ROBOT_CENTER_TO_FACE + 5)/SQRT_2;
        pose = new Pose(app_camp_adapt_distance(x), y, app_camp_adapt_angle(45), LOW_SPEED_LINEAR, LOW_SPEED_ANGULAR, false);
        pose->set_before_pose(std::bind(&DropInShedAction::enable_shed_obstacle, this, false));
        poses_->push_back(pose);

        // Step back
        x = statuette_center_on_pedestal.x();
        y = statuette_center_on_pedestal.y();
        x -= (APP_SAMPLE_RADIUS + 5)/SQRT_2;
        y += (APP_SAMPLE_RADIUS + 5)/SQRT_2;
        x -= (APP_SAMPLE_RADIUS + 5)/SQRT_2;
        y -= (APP_SAMPLE_RADIUS + 5)/SQRT_2;
        x -= (APP_SAMPLE_RADIUS*3 + 10)/SQRT_2;
        y -= (APP_SAMPLE_RADIUS*3 + 10)/SQRT_2;
        pose = new Pose(app_camp_adapt_distance(x), y, app_camp_adapt_angle(45), MAX_SPEED_LINEAR, MAX_SPEED_ANGULAR);
        pose->set_after_pose(std::bind(&DropInShedAction::enable_shed_obstacle, this, true));
        poses_->push_back(pose);

        // Drop middle sample
        x = statuette_center_on_pedestal.x();
        y = statuette_center_on_pedestal.y();
        x -= (APP_SAMPLE_RADIUS*3 + 5)/SQRT_2;
        y -= (APP_SAMPLE_RADIUS*3 + 5)/SQRT_2;
        x -= (CENTRAL_ARM_LENGTH_DOWN + ROBOT_CENTER_TO_FACE + 5)/SQRT_2;
        y -= (CENTRAL_ARM_LENGTH_DOWN + ROBOT_CENTER_TO_FACE + 5)/SQRT_2;
        pose = new Pose(app_camp_adapt_distance(x), y, app_camp_adapt_angle(45), MAX_SPEED_LINEAR, MAX_SPEED_ANGULAR);
        pose->set_after_pose(std::bind(&DropInShedAction::drop_sample, this));
        poses_->push_back(pose);

        // Push middle sample
        x = statuette_center_on_pedestal.x();
        y = statuette_center_on_pedestal.y();
        x -= (ROBOT_CENTER_TO_FACE + 5)/SQRT_2;
        y -= (ROBOT_CENTER_TO_FACE + 5)/SQRT_2;
        pose = new Pose(app_camp_adapt_distance(x), y, app_camp_adapt_angle(45), LOW_SPEED_LINEAR, LOW_SPEED_ANGULAR, false);
        pose->set_before_pose(std::bind(&DropInShedAction::enable_shed_obstacle, this, false));
        poses_->push_back(pose);

        // Step back
        x = statuette_center_on_pedestal.x();
        y = statuette_center_on_pedestal.y();
        x -= (APP_SAMPLE_RADIUS*3 + 10)/SQRT_2;
        y -= (APP_SAMPLE_RADIUS*3 + 10)/SQRT_2;
        x -= (CENTRAL_ARM_LENGTH_DOWN + ROBOT_CENTER_TO_FACE + 5)/SQRT_2;
        y -= (CENTRAL_ARM_LENGTH_DOWN + ROBOT_CENTER_TO_FACE + 5)/SQRT_2;
        pose = new Pose(app_camp_adapt_distance(x), y, app_camp_adapt_angle(45), MAX_SPEED_LINEAR, MAX_SPEED_ANGULAR);
        pose->set_after_pose(std::bind(&DropInShedAction::enable_shed_obstacle, this, true));
        poses_->push_back(pose);

        pose_it_ = poses_->begin();
    };

    ~DropInShedAction() {};

    float weight() const override {
        if (app_get_context().samples_in_robot.size() < 3) {
            return 0;
        }
        return default_weight_;
    };

    void drop_sample() {
        // std::cout << "DropInShedAction::drop_sample" << std::endl;
        // TODO: Drop sample with center arm
        Sample *sample = app_get_context().samples_in_robot.top();
        sample->set_loc(SampleLocation::Dropped);
        app_get_context().samples_in_robot.pop();
    };

    void enable_shed_obstacle(bool enable) {
        // std::cout << "DropInShedAction::enable_shed_obstacle(" << enable << ")" << std::endl;
        app_get_shed_obstacles()[app_camp_get_color()]->enable(enable);
    };

    void after_action() override {
        app_get_context().score += 10;
    };

private:
    SampleColor color_;
    float default_weight_;
    double x_;
};

class ExcavationSquaresAction: public Action {
public:
    ExcavationSquaresAction(
        float weight
    ) : Action("Excavation squares"),
        default_weight_(weight), at_leat_one_flip_(false)
    {
        Pose *pose;
        y_ = 2000 - ROBOT_CENTER_TO_SIDE - 30;
        angle_ = app_camp_adapt_angle(180);

        // First square
        pose = new Pose(app_camp_adapt_distance(832.5), y_, angle_, MAX_SPEED_LINEAR, MAX_SPEED_ANGULAR, false);
        pose->set_before_pose(std::bind(&ExcavationSquaresAction::foo, this));
        pose->set_after_pose(std::bind(&ExcavationSquaresAction::test1, this));
        poses_->push_back(pose);

        pose_it_ = poses_->begin();
    };

    float weight() const override {
        return default_weight_;
    };

    void foo() {
        // std::cout << "ExcavationSquaresAction::foo" << std::endl;
    };

    void test1() {
        // std::cout << "ExcavationSquaresAction::test1" << std::endl;
        // TODO: get value
        // results values:
        //   - 0: yellow
        //   - 1: purple
        //   - 2: red
        //   - 3: no square (already flipped)
        int result = 0;
        if (result == as_number(app_camp_get_color())) {
            // std::cout << "ExcavationSquaresAction::test1: Own square detected" << std::endl;
            flip_square();
            flip_next(647.5);
        }
        else {
            // std::cout << "ExcavationSquaresAction::test1: Opponent square detected" << std::endl;
            flip_next(647.5);
            flip_next(1037.5);
        }

        // Test 2
        double x = 277.5;
        Pose *pose = new Pose(app_camp_adapt_distance(x), y_, angle_, MAX_SPEED_LINEAR, MAX_SPEED_ANGULAR, false);
        pose->set_after_pose(std::bind(&ExcavationSquaresAction::test2, this));
        Pose *cur_pose = current_pose();
        poses_->push_back(pose);
        pose_it_ = std::find(poses_->begin(), poses_->end(), cur_pose);
    };

    /// 277.5 92.5 -92.5 -277.5
    void test2() {
        // std::cout << "ExcavationSquaresAction::test2" << std::endl;
        int result = 0;
        if (result == as_number(app_camp_get_color())) {
            // std::cout << "ExcavationSquaresAction::test2: Own square detected" << std::endl;
            flip_square();
            flip_next(-277.5);
        }
        else {
            // std::cout << "ExcavationSquaresAction::test2: Opponent square detected" << std::endl;
            flip_next(92.5);
            flip_next(-92.5);
        }
    };

    void flip_next(double x) {
        // std::cout << "ExcavationSquaresAction::flip_next(" << x << ")" << std::endl;
        Pose *pose = new Pose(app_camp_adapt_distance(x), y_, angle_, MAX_SPEED_LINEAR, MAX_SPEED_ANGULAR, false);
        pose->set_after_pose(std::bind(&ExcavationSquaresAction::flip_square, this));
        Pose *cur_pose = current_pose();
        poses_->push_back(pose);
        pose_it_ = std::find(poses_->begin(), poses_->end(), cur_pose);
    };

    void flip_square() {
        // std::cout << "ExcavationSquaresAction::flip_square" << std::endl;
        // TODO
        if (! at_leat_one_flip_) {
            app_get_context().score += 5;
            at_leat_one_flip_ = true;
        }
        app_get_context().score += 5;
    };

private:
    float default_weight_;
    bool at_leat_one_flip_;
    double y_;
    double angle_;
};

class PushAndEndAction: public Action {
public:
    PushAndEndAction(): Action("Push and end action") {
        Pose *pose = new Pose(
            app_camp_adapt_distance(50),
            675,
            app_camp_adapt_angle(0),
            NORMAL_SPEED_LINEAR, NORMAL_SPEED_ANGULAR
        );
        pose->set_after_pose(std::bind(&PushAndEndAction::before_push, this));
        poses_->push_back(pose);

        pose = new Pose(
            app_camp_adapt_distance(1500 - APP_SAMPLE_RADIUS*2 - ROBOT_CENTER_TO_FACE - 40),
            675,
            -90,
            NORMAL_SPEED_LINEAR, NORMAL_SPEED_ANGULAR, false
        );
        poses_->push_back(pose);

        pose_it_ = poses_->begin();
    };
    ~PushAndEndAction() {};

    void before_push() {
        // std::cout << "PushAndEndAction::before_push" << std::endl;
        app_samples_get_one(SampleId::TableFixedBlue)->obstacle()->enable(false);
        app_samples_get_one(SampleId::TableFixedRed)->obstacle()->enable(false);
        app_samples_get_one(SampleId::TableFixedGreen)->obstacle()->enable(false);
    };

    void after_action() override {
        // std::cout << "PushAndEndAction::after_action" << std::endl;
        app_get_context().score += 1; // 1 sample
        app_get_context().score += 20;
        if (! _score_sent) {
            _pb_score.set_score(app_get_context().score);
            pf_get_uartpb()->send_message(_pb_score);
            _score_sent = true;
        }
    };

    float weight() const override {
        return 1;
    };
};

static void _app_actions_init_game()
{
    _actions = new Actions();

    // Start
    _actions->push_back(new StartAction());

    // // End
    // _actions->push_back(new ExcavationEndAction());

    // // Get fixed sample on table
    // _actions->push_back(new GetFixedSampleAction(
    //     SampleId::TableFixedRed,
    //     { SampleId::TableFixedGreen, SampleId::TableFixedBlue },
    //     1000
    // ));
    // _actions->push_back(new GetFixedSampleAction(
    //     SampleId::TableFixedGreen,
    //     { SampleId::TableFixedRed, SampleId::TableFixedBlue },
    //     1000
    // ));
    // _actions->push_back(new GetFixedSampleAction(
    //     SampleId::TableFixedBlue,
    //     { SampleId::TableFixedRed, SampleId::TableFixedGreen },
    //     1000
    // ));

    // // Detect random samples
    // _actions->push_back(new DetectRandomSamplesAction(900));
    // // _actions->push_back(new DetectRandomSamplesAction(100, true));

    // // Drop samples on gallery
    // _actions->push_back(new DropInGalleryAction(SampleColor::Red, 800));
    // _actions->push_back(new DropInGalleryAction(SampleColor::Red, 800));
    // _actions->push_back(new DropInGalleryAction(SampleColor::Green, 800));
    // _actions->push_back(new DropInGalleryAction(SampleColor::Green, 800));
    // _actions->push_back(new DropInGalleryAction(SampleColor::Blue, 800));
    // _actions->push_back(new DropInGalleryAction(SampleColor::Blue, 800));

    // Statuette only actions
    _actions->push_back(new StatuetteOnlyAction(700));

    // // Drop in sched
    // _actions->push_back(new DropInShedAction(600));

    // _actions->push_back(new ExcavationSquaresAction(500));

    // // Get sample on top dispenser
    // _actions->push_back(new SideDispenserAction(400));
    // // _actions->push_back(new TopDispenserAction(400));

    // Drop in camp (300)

    // Push fixed samples in camp and stop
    _actions->push_back(new PushAndEndAction());
}

static void _app_actions_init_approval()
{
    _actions = new Actions();

    _actions->push_back(new ApprovalAction());
}

void app_actions_init()
{
    if (_actions) {
        delete _actions;
        _actions = nullptr;
    }

    switch (_strategy) {
        case ActionStrategy::Approval:
            _app_actions_init_approval();
            break;
        case ActionStrategy::Game:
            _app_actions_init_game();
            break;
    }
}

void app_actions_set_strategy(ActionStrategy strategy)
{
    _strategy = strategy;
}

Actions * app_actions_get()
{
    if (! _actions) {
        app_actions_init();
    }
    return _actions;
}

void app_actions_reset() {
    if (_actions) {
        delete _actions;
        _actions = nullptr;
    }
    app_actions_get();
}

} // namespace app

} // namespace cogip
