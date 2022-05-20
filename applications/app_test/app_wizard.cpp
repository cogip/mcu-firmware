#include "platform.hpp"

#include <iostream>

namespace cogip {

namespace app {

static cogip::wizard::Wizard::PB_Message wizard_message;

void app_wizard(void)
{
    cogip::wizard::Wizard *wizard = pf_get_wizard();

    wizard_message.clear();
    wizard_message.mutable_name() = "Skip Wizard example?";
    wizard_message.mutable_boolean().set_value(true);
    wizard_message = wizard->request(wizard_message);
    if (wizard_message.has_boolean()) {
        if (wizard_message.get_boolean().value()) {
            puts("Wizard skipped");
            return;
        }
        puts("Continue wizard");
    }
    else {
        puts("Wizard error: bad response type.");
    }

    wizard_message.clear();
    wizard_message.mutable_name() = "Enter an integer";
    wizard_message.mutable_integer().set_value(2);
    wizard_message = wizard->request(wizard_message);
    if (wizard_message.has_integer()) {
        printf("Wizard integer response: %" PRId32 "\n", wizard_message.get_integer().value());
    }
    else {
        puts("Wizard error: bad response type.");
    }

    wizard_message.clear();
    wizard_message.mutable_name() = "Enter a float";
    wizard_message.mutable_floating().set_value(2.5);
    wizard_message = wizard->request(wizard_message);
    if (wizard_message.has_floating()) {
        printf("Wizard float response: %f\n", wizard_message.get_floating().value());
    }
    else {
        puts("Wizard error: bad response type.");
    }

    wizard_message.clear();
    wizard_message.mutable_name() = "Enter a string";
    wizard_message.mutable_str().mutable_value() = "foo";
    wizard_message = wizard->request(wizard_message);
    if (wizard_message.has_str()) {
        printf("Wizard string response: %s\n", wizard_message.get_str().value());
    }
    else {
        puts("Wizard error: bad response type.");
    }

    wizard_message.clear();
    wizard_message.mutable_name() = "Warning";
    wizard_message.mutable_message().mutable_value() = "Please insert starter";
    wizard_message = wizard->request(wizard_message);
    if (wizard_message.has_message()) {
        puts("Wizard message acknowledged");
    }
    else {
        puts("Wizard error: bad response type.");
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
        printf("Wizard choice integer response: %" PRId32 "\n", wizard_message.get_choice_integer().value());
    }
    else {
        puts("Wizard error: bad response type.");
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
        printf("Wizard choice floating response: %f\n", wizard_message.get_choice_floating().value());
    }
    else {
        puts("Wizard error: bad response type.");
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
        printf("Wizard choice string response: %s\n", wizard_message.get_choice_str().value());
    }
    else {
        puts("Wizard error: bad response type.");
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
        printf("Wizard select integer response:");
        for(uint32_t i = 0; i < wizard_message.get_select_integer().value().get_length(); ++i) {
          printf(" %" PRId32, wizard_message.get_select_integer().value().get_const(i).get());
        }
        puts("");
    }
    else {
        puts("Wizard error: bad response type.");
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
        printf("Wizard select floating response:");
        for(uint32_t i = 0; i < wizard_message.get_select_floating().value().get_length(); ++i) {
          printf(" %f", wizard_message.get_select_floating().value().get_const(i).get());
        }
        puts("");
    }
    else {
        puts("Wizard error: bad response type.");
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
        printf("Wizard select string response:");
        for(uint32_t i = 0; i < wizard_message.get_select_str().value().get_length(); ++i) {
          printf(" %s", wizard_message.get_select_str().value().get_const(i).get_const());
        }
        puts("");
    }
    else {
        puts("Wizard error: bad response type.");
    }

    wizard_message.clear();
    wizard_message.mutable_name() = "Choose/Verify your camp";
    wizard_message.mutable_camp().mutable_value() = "yellow";
    wizard_message = wizard->request(wizard_message);
    if (wizard_message.has_camp()) {
        printf("Wizard camp response: %s\n", wizard_message.get_camp().value());
    }
    else {
        puts("Wizard error: bad response type.");
    }

    wizard_message.clear();
    wizard_message.mutable_name() = "Check camera focus";
    wizard_message.mutable_camera().set_value(true);
    wizard_message = wizard->request(wizard_message);
    if (wizard_message.has_camera()) {
        puts("Camera checked");
    }
    else {
        puts("Wizard error: bad response type.");
    }

    puts("Wizard done");
}

} // namespace app

} // namespace cogip
