#include "actuator/LiftsLimitSwitchesManager.hpp"
#include "log.h"
#include <inttypes.h>
#include <thread.h>

#define ENABLE_DEBUG 1
#include <debug.h>

namespace cogip {
namespace actuators {
namespace positional_actuators {

LiftsLimitSwitchesManager& LiftsLimitSwitchesManager::instance()
{
    static LiftsLimitSwitchesManager inst;
    return inst;
}

void LiftsLimitSwitchesManager::init()
{
    // Create the thread to handle GPIO events
    event_thread_pid_ =
        thread_create(event_stack_, sizeof(event_stack_), THREAD_PRIORITY_MAIN - 1,
                      THREAD_CREATE_STACKTEST, event_thread_entry, nullptr, "gpio_evt");

    LOG_INFO("Motor limit switches manager started\n");
}

int LiftsLimitSwitchesManager::register_gpio(gpio_t pin, Lift* lift)
{
    if (pin == GPIO_UNDEF)
        return -ENODEV;

    mutex_lock(&mutex_);

    // Check if already registered
    if (gpio_to_event_.find(pin) != gpio_to_event_.end()) {
        LOG_INFO("GPIO %p already registered, updating callback\n", reinterpret_cast<void*>(pin));
        callbacks_[pin] = lift;
        mutex_unlock(&mutex_);
        return 0;
    }

    // Read initial state before configuring interrupt
    gpio_init(pin, GPIO_IN_PU);
    int initial_state = gpio_read(pin);
    LOG_INFO("GPIO %p initial state (with pull-up): %d\n", reinterpret_cast<void*>(pin),
             initial_state);

    // Initialize pin interrupt on both edges to catch any transition
    int ret = gpio_init_int(pin, GPIO_IN_PU, GPIO_BOTH, isr_callback, reinterpret_cast<void*>(pin));
    if (ret != 0) {
        LOG_ERROR("Failed to init pin %p, error: %d\n", reinterpret_cast<void*>(pin), ret);

        mutex_unlock(&mutex_);

        return -EIO;
    }
    LOG_INFO("GPIO %p interrupt configured successfully\n", reinterpret_cast<void*>(pin));

    // Allocate a new event from the pool
    event_t* evt = event_pool_.create();
    if (!evt) {
        LOG_ERROR("Cannot allocate event for pin %p\n", reinterpret_cast<void*>(pin));

        mutex_unlock(&mutex_);

        return -ENOMEM;
    }

    evt->handler = event_handler;
    evt->list_node.next = nullptr;

    gpio_to_event_[pin] = evt;
    event_to_gpio_[evt] = pin;
    callbacks_[pin] = lift;

    LOG_INFO("GPIO %p registered\n", reinterpret_cast<void*>(pin));

    mutex_unlock(&mutex_);

    return 0;
}

int LiftsLimitSwitchesManager::unregister_gpio(gpio_t pin)
{
    if (pin == GPIO_UNDEF)
        return -ENODEV;

    mutex_lock(&mutex_);

    auto it = gpio_to_event_.find(pin);
    if (it == gpio_to_event_.end()) {
        mutex_unlock(&mutex_);
        return -ENOENT; // GPIO not registered
    }

    event_t* evt = it->second;

    // Disable interrupt on the pin
    gpio_irq_disable(pin);

    // Clean up maps
    gpio_to_event_.erase(pin);
    event_to_gpio_.erase(evt);
    callbacks_.erase(pin);

    // Free the event
    event_pool_.release(evt);

    LOG_INFO("GPIO %p unregistered\n", reinterpret_cast<void*>(pin));

    mutex_unlock(&mutex_);

    return 0;
}

void LiftsLimitSwitchesManager::isr_callback(void* arg)
{
    gpio_t pin = reinterpret_cast<gpio_t>(arg);
    event_t* evt = instance().gpio_to_event_[pin];

    if (evt) {
        event_post(&instance().event_queue_, evt);
        DEBUG("Event %p thrown\n", static_cast<void*>(evt));
    }
}

void LiftsLimitSwitchesManager::event_handler(event_t* evt)
{
    DEBUG("Event %p handling\n", static_cast<void*>(evt));

    gpio_t pin = instance().event_to_gpio_[evt];
    instance().handle_gpio_event(pin);
}

void LiftsLimitSwitchesManager::handle_gpio_event(gpio_t pin)
{
    DEBUG("GPIO %p triggered\n", reinterpret_cast<void*>(pin));
    mutex_lock(&mutex_);

    auto it = callbacks_.find(pin);
    if (it != callbacks_.end()) {
        it->second->at_limits(pin); // Call user-registered callback
    }

    mutex_unlock(&mutex_);

    LOG_INFO("Event handled\n");
}

void* LiftsLimitSwitchesManager::event_thread_entry(void*)
{
    // Initialize the queue in the same thread than the event loop
    event_queue_init(&instance().event_queue_);

    // Main event loop (blocks waiting for events)
    LOG_INFO("Start event loop\n");
    event_loop(&instance().event_queue_);
    return nullptr;
}

} // namespace positional_actuators
} // namespace actuators
} // namespace cogip
