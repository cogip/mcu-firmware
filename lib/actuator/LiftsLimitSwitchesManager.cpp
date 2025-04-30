#include "actuator/LiftsLimitSwitchesManager.hpp"
#include <thread.h>
#include <iostream>

namespace cogip {
namespace actuators {
namespace positional_actuators {

LiftsLimitSwitchesManager& LiftsLimitSwitchesManager::instance() {
    static LiftsLimitSwitchesManager inst;
    return inst;
}

void LiftsLimitSwitchesManager::init() {
    // Create the thread to handle GPIO events
    event_thread_pid_ = thread_create(
        event_stack_, sizeof(event_stack_),
        THREAD_PRIORITY_MAIN - 1,
        THREAD_CREATE_STACKTEST,
        event_thread_entry, nullptr, "gpio_evt"
    );

    std::cout << "Motor limit switches manager started" << std::endl;
}

int LiftsLimitSwitchesManager::register_gpio(gpio_t pin, cogip::actuators::positional_actuators::Lift* lift) {
    if (pin == GPIO_UNDEF) return -ENODEV;

    mutex_lock(&mutex_);
    // Initialize pin interrupt on falling edges with pulldown
    if (gpio_init_int(pin, GPIO_IN, GPIO_FALLING, isr_callback, (void*)pin) != 0) {
        std::cerr << "ERROR: Failed to init pin " << pin << std::endl;

        mutex_unlock(&mutex_);

        return -EIO;
    }

    // Allocate a new event from the pool
    event_t* evt = event_pool_.create();
    if (!evt) {
        std::cerr << "ERROR: Cannot allocate event for pin " << pin << std::endl;

        mutex_unlock(&mutex_);

        return -ENOMEM;
    }

    evt->handler = event_handler;
    evt->list_node.next = nullptr;

    gpio_to_event_[pin] = evt;
    event_to_gpio_[evt] = pin;
    callbacks_[pin] = lift;

    // Initialize pin interrupt on falling edges with pulldown
    if (gpio_init_int(pin, GPIO_IN, GPIO_FALLING, isr_callback, (void*)pin) != 0) {
        std::cerr << "ERROR: Failed to init pin " << pin << std::endl;

        return -EIO;
    }

    std::cout << "GPIO " << pin << " registered" << std::endl;

    mutex_unlock(&mutex_);

    return 0;
}

void LiftsLimitSwitchesManager::isr_callback(void* arg) {
    gpio_t pin = reinterpret_cast<gpio_t>(arg);
    event_t* evt = instance().gpio_to_event_[pin];

    if (evt) {
        event_post(&instance().event_queue_, evt);
        std::cout << "Event " << evt << " thrown" << std::endl;
    }
}

void LiftsLimitSwitchesManager::event_handler(event_t* evt) {
    std::cout << "Event " << evt << " handling" << std::endl;

    gpio_t pin = instance().event_to_gpio_[evt];
    instance().handle_gpio_event(pin);
}

void LiftsLimitSwitchesManager::handle_gpio_event(gpio_t pin) {
    std::cout << "GPIO " << pin << " triggered" << std::endl;
    mutex_lock(&mutex_);

    auto it = callbacks_.find(pin);
    if (it != callbacks_.end()) {
        it->second->at_limits(pin); // Call user-registered callback
    }

    mutex_unlock(&mutex_);

    std::cout << "Event handled" << std::endl;
}

void* LiftsLimitSwitchesManager::event_thread_entry(void*) {
    // Initialize the queue in the same thread than the event loop
    event_queue_init(&instance().event_queue_);

    // Main event loop (blocks waiting for events)
    std::cout << "Start event loop" << std::endl;
    event_loop(&instance().event_queue_);
    return nullptr;
}

} // namespace positional_actuators
} // namespace actuators
} // namespace cogip
