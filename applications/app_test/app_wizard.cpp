#include "app_camp.hpp"
#include "platform.hpp"

#include <iostream>

namespace cogip {

namespace app {

static cogip::wizard::Wizard::PB_Message wizard_message;

void app_wizard(void)
{
    cogip::wizard::Wizard *wizard = pf_get_wizard();

    wizard_message.clear();
    wizard_message.mutable_name() = "Choose your camp";
    wizard_message.mutable_camp().mutable_value() = "yellow";
    wizard_message = wizard->request(wizard_message);
    if (wizard_message.has_camp()) {
        std::string select_camp = wizard_message.get_camp().value();
        COGIP_DEBUG_COUT("Selected camp: " << select_camp);
        app_camp_set_color(select_camp == "yellow" ? CampColor::Yellow : CampColor::Purple);
    }
    else {
        COGIP_DEBUG_COUT("Wizard error: bad response type for camp color.");
    }

    wizard_message.clear();
    wizard_message.mutable_name() = "Check camera focus";
    wizard_message.mutable_camera().set_value(true);
    wizard_message = wizard->request(wizard_message);
    if (wizard_message.has_camera()) {
        COGIP_DEBUG_COUT("Camera checked");
    }
    else {
        COGIP_DEBUG_COUT("Wizard error: bad response type fro camera focus.");
    }

    wizard_message.clear();
    wizard_message.mutable_name() = "Message";
    wizard_message.mutable_message().mutable_value() = "Ready to start?";
    wizard_message = wizard->request(wizard_message);
    if (wizard_message.has_message()) {
        COGIP_DEBUG_COUT("Wizard start message acknowledged");
    }
    else {
        COGIP_DEBUG_COUT("Wizard error: bad response type for message start.");
    }

    pf_get_planner()->set_allow_change_path_pose(true);
    pf_get_planner()->start();

#if 0  // Keep unused code as examples for other types supported by wizard.
    wizard_message.clear();
    wizard_message.mutable_name() = "Skip Wizard example?";
    wizard_message.mutable_boolean().set_value(true);
    wizard_message = wizard->request(wizard_message);
    if (wizard_message.has_boolean()) {
        if (wizard_message.get_boolean().value()) {
            COGIP_DEBUG_COUT("Wizard skipped");
            return;
        }
        COGIP_DEBUG_COUT("Continue wizard");
    }
    else {
        COGIP_DEBUG_COUT("Wizard error: bad response type.");
    }

    wizard_message.clear();
    wizard_message.mutable_name() = "Enter an integer";
    wizard_message.mutable_integer().set_value(2);
    wizard_message = wizard->request(wizard_message);
    if (wizard_message.has_integer()) {
        COGIP_DEBUG_COUT("Wizard integer response: " << wizard_message.get_integer().value());
    }
    else {
        COGIP_DEBUG_COUT("Wizard error: bad response type.");
    }

    wizard_message.clear();
    wizard_message.mutable_name() = "Enter a float";
    wizard_message.mutable_floating().set_value(2.5);
    wizard_message = wizard->request(wizard_message);
    if (wizard_message.has_floating()) {
        COGIP_DEBUG_COUT("Wizard float response: " << wizard_message.get_floating().value());
    }
    else {
        COGIP_DEBUG_COUT("Wizard error: bad response type.");
    }

    wizard_message.clear();
    wizard_message.mutable_name() = "Enter a string";
    wizard_message.mutable_str().mutable_value() = "foo";
    wizard_message = wizard->request(wizard_message);
    if (wizard_message.has_str()) {
        COGIP_DEBUG_COUT("Wizard string response: " << wizard_message.get_str().value());
    }
    else {
        COGIP_DEBUG_COUT("Wizard error: bad response type.");
    }

    wizard_message.clear();
    wizard_message.mutable_name() = "Choose a integer value";
    auto &choice_integer = wizard_message.mutable_choice_integer();
    choice_integer.mutable_choices().add(1);
    choice_integer.mutable_choices().add(2);
    choice_integer.mutable_choices().add(3);
    choice_integer.mutable_value() = 2;
    wizard_message = wizard->request(wizard_message);
    if (wizard_message.has_choice_integer()) {
        COGIP_DEBUG_COUT("Wizard choice integer response: " << wizard_message.get_choice_integer().value());
    }
    else {
        COGIP_DEBUG_COUT("Wizard error: bad response type.");
    }

    wizard_message.clear();
    wizard_message.mutable_name() = "Choose a float value";
    auto &choice_floating = wizard_message.mutable_choice_floating();
    choice_floating.mutable_choices().add(1.5);
    choice_floating.mutable_choices().add(2.5);
    choice_floating.mutable_choices().add(3.5);
    choice_floating.mutable_value() = 2.5;
    wizard_message = wizard->request(wizard_message);
    if (wizard_message.has_choice_floating()) {
        COGIP_DEBUG_COUT("Wizard choice floating response: " << wizard_message.get_choice_floating().value());
    }
    else {
        COGIP_DEBUG_COUT("Wizard error: bad response type.");
    }

    wizard_message.clear();
    wizard_message.mutable_name() = "Choose a string value";
    auto &choice_str = wizard_message.mutable_choice_str();
    choice_str.mutable_choices().get(0) = "foo";
    choice_str.mutable_choices().get(1) = "bar";
    choice_str.mutable_choices().get(2) = "baz";
    choice_str.mutable_value() = "bar";
    wizard_message = wizard->request(wizard_message);
    if (wizard_message.has_choice_str()) {
        COGIP_DEBUG_COUT("Wizard choice string response: " << wizard_message.get_choice_str().value());
    }
    else {
        COGIP_DEBUG_COUT("Wizard error: bad response type.");
    }

    wizard_message.clear();
    wizard_message.mutable_name() = "Select one or multiple integer values";
    auto &select_integer = wizard_message.mutable_select_integer();
    select_integer.mutable_choices().add(1);
    select_integer.mutable_choices().add(2);
    select_integer.mutable_choices().add(3);
    select_integer.mutable_value().add(1);
    select_integer.mutable_value().add(3);
    wizard_message = wizard->request(wizard_message);
    if (wizard_message.has_select_integer()) {
        COGIP_DEBUG_COUT("Wizard select integer response:");
        for(uint32_t i = 0; i < wizard_message.get_select_integer().value().get_length(); ++i) {
          COGIP_DEBUG_COUT(" " << wizard_message.get_select_integer().value().get_const(i).get());
        }
        COGIP_DEBUG_COUT("");
    }
    else {
        COGIP_DEBUG_COUT("Wizard error: bad response type.");
    }

    wizard_message.clear();
    wizard_message.mutable_name() = "Select one or multiple float values";
    auto &select_floating = wizard_message.mutable_select_floating();
    select_floating.mutable_choices().add(1.5);
    select_floating.mutable_choices().add(2.5);
    select_floating.mutable_choices().add(3.5);
    select_floating.mutable_value().add(1.5);
    select_floating.mutable_value().add(3.5);
    wizard_message = wizard->request(wizard_message);
    if (wizard_message.has_select_floating()) {
        COGIP_DEBUG_COUT("Wizard select floating response:");
        for(uint32_t i = 0; i < wizard_message.get_select_floating().value().get_length(); ++i) {
          COGIP_DEBUG_COUT(" " << wizard_message.get_select_floating().value().get_const(i).get());
        }
        COGIP_DEBUG_COUT("");
    }
    else {
        COGIP_DEBUG_COUT("Wizard error: bad response type.");
    }

    wizard_message.clear();
    wizard_message.mutable_name() = "Select one or multiple string values";
    auto &select_str = wizard_message.mutable_select_str();
    select_str.mutable_choices().get(0) = "foo";
    select_str.mutable_choices().get(1) = "bar";
    select_str.mutable_choices().get(2) = "baz";
    select_str.mutable_value().get(0) = "foo";
    select_str.mutable_value().get(1) = "baz";
    wizard_message = wizard->request(wizard_message);
    if (wizard_message.has_select_str()) {
        COGIP_DEBUG_COUT("Wizard select string response:");
        for(uint32_t i = 0; i < wizard_message.get_select_str().value().get_length(); ++i) {
          COGIP_DEBUG_COUT(" " << wizard_message.get_select_str().value().get_const(i).get_const());
        }
        COGIP_DEBUG_COUT("");
    }
    else {
        COGIP_DEBUG_COUT("Wizard error: bad response type.");
    }
#endif

    COGIP_DEBUG_COUT("Wizard done");
}

} // namespace app

} // namespace cogip
