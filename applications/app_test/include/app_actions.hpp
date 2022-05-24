#pragma once

#include <iostream>
#include <list>
#include <string>

#include "app_pose.hpp"

namespace cogip {

namespace app {

enum class ActionStrategy {
    Approval,
    Game
};

class Action {
public:
    Action(
        const std::string &name,
        uint8_t direct_score = 0,
        uint8_t potential_score = 0
        );
    virtual ~Action();

    const std::string & name() { return name_; };
    virtual Poses & poses() { return *poses_; };
    virtual float weight() const { return direct_score_ + 0.5 * potential_score_; };
    virtual void before_action() {};
    virtual void after_action() {};
    virtual Pose *current_pose() { return *pose_it_; };
    virtual Poses::iterator end_pose() { return poses_->end(); };
    virtual Poses::iterator next_pose() {
        if (pose_it_ != poses_->end()) {
            pose_it_++;
        }
        if (pose_it_ != poses_->end()) {
            (*pose_it_)->before_pose();
        }
        return pose_it_;
    };

protected:
    Poses::iterator pose_it_;
    std::string name_;
    Poses *poses_;
    uint8_t direct_score_;
    uint8_t potential_score_;
};

class Actions: public std::list<class Action *> {
public:
    Actions() : current_(nullptr) {};
    virtual ~Actions();

    Action *new_action(Action *recycle_action=nullptr);

private:
    Action *current_;
};

void app_actions_init(ActionStrategy strategy);

void app_actions_set_strategy(ActionStrategy strategy);

Actions * app_actions_get();

void app_actions_reset();

} // namespace app

} // namespace cogip
