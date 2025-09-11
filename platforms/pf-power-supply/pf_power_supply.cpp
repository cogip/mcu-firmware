///  Copyright (C) 2025 COGIP Robotics association
///
///  This file is subject to the terms and conditions of the GNU Lesser
///  General Public License v2.1. See the file LICENSE in the top level
///  directory for more details.

///  @ingroup     platforms
///  @brief       power supply platform
///
///  @author      Mathis LECRIVAIN <lecrivain.mathis@gmail.com>

// RIOT includes
#include "log.h"
#include <msg.h>
#include <mutex.h>
#include <thread.h>

// Platform includes
#include "pf_power_supply.hpp"
#include "platform.hpp"

// Libraries includes
#include "gpio/gpio.hpp"

// CAN protobuf includes
#include "canpb/ReadBuffer.hpp"

// Import protobuf messages (generated files will be in the build directory)
#include "PB_PowerSupply.hpp"

namespace cogip {
namespace pf {
namespace power_supply {

/// GPIO array indices for clear identification
enum gpio_index_t {
    GPIO_INDEX_P3V3_PGOOD = 0,
    GPIO_INDEX_P5V0_PGOOD,
    GPIO_INDEX_P7V5_PGOOD,
    GPIO_INDEX_PxVx_PGOOD,
    GPIO_INDEX_EMERGENCY_STOP,
    GPIO_INDEX_BATTERY_VALID_N,
    GPIO_INDEX_DC_SUPPLY_VALID_N
};

/// @brief GPIO info structure for factorized handling
/// @details Contains all necessary information to manage a GPIO in a unified way
struct gpio_info_t
{
    const gpio::GPIO& gpio; ///< Reference to the GPIO object
    bool& state_ref;        ///< Reference to the boolean state variable
    const char* name;       ///< Human-readable name for logging
};

/// GPIOs handler thread stack
static char _gpio_handling_thread_stack[THREAD_STACKSIZE_DEFAULT];

/// Thread for GPIO handling
static kernel_pid_t _gpio_thread_pid = KERNEL_PID_UNDEF;

/// Mutex for protecting gpio_states_ access from multiple threads (internal task + canpb task)
static mutex_t _gpio_states_mutex = MUTEX_INIT;

/// @brief GPIO states structure
/// @details Stores the current state of all power supply GPIO signals
static struct
{
    bool p3V3_pgood;      ///< 3.3V power good status (true = OK, false = fault)
    bool p5V0_pgood;      ///< 5.0V power good status (true = OK, false = fault)
    bool p7V5_pgood;      ///< 7.5V power good status (true = OK, false = fault)
    bool pxVx_pgood;      ///< Variable voltage power good status (true = OK, false = fault)
    bool emergency_stop;  ///< Emergency stop button status (false = engaged, true = released)
    bool battery_valid;   ///< Battery valid status (true = connected, false = disconnected)
    bool dc_supply_valid; ///< DC supply valid status (true = connected, false = disconnected)
} gpio_states_ = {
    .p3V3_pgood = false,
    .p5V0_pgood = false,
    .p7V5_pgood = false,
    .pxVx_pgood = false,
    .emergency_stop = false,
    .battery_valid = false,
    .dc_supply_valid = false,
};

// Forward declaration needed for GPIO object initialization
static void gpio_change_cb_(void* arg);

// PGood GPIO objects creation
static gpio::GPIO p3V3_pgood_gpio_(P3V3_PGOOD_PIN, GPIO_IN, GPIO_BOTH, gpio_change_cb_,
                                   (void*)GPIO_INDEX_P3V3_PGOOD);
static gpio::GPIO p5V0_pgood_gpio_(P5V0_PGOOD_PIN, GPIO_IN, GPIO_BOTH, gpio_change_cb_,
                                   (void*)GPIO_INDEX_P5V0_PGOOD);
static gpio::GPIO p7V5_pgood_gpio_(P7V5_PGOOD_PIN, GPIO_IN, GPIO_BOTH, gpio_change_cb_,
                                   (void*)GPIO_INDEX_P7V5_PGOOD);
static gpio::GPIO pxVx_pgood_gpio_(PxVx_PGOOD_PIN, GPIO_IN, GPIO_BOTH, gpio_change_cb_,
                                   (void*)GPIO_INDEX_PxVx_PGOOD);

// Emergency stop GPIO objects
static gpio::GPIO emergency_stop_gpio_(EN_HIGH_POWER_PIN, GPIO_IN, GPIO_BOTH, gpio_change_cb_,
                                       (void*)GPIO_INDEX_EMERGENCY_STOP);

// Power source GPIO objects - VALID_N pins are active low
static gpio::GPIO battery_valid_n_gpio_(BATTERY_VALID_N_PIN, GPIO_IN, GPIO_BOTH, gpio_change_cb_,
                                        (void*)GPIO_INDEX_BATTERY_VALID_N);
static gpio::GPIO dc_supply_valid_n_gpio_(DC_SUPPLY_VALID_N_PIN, GPIO_IN, GPIO_BOTH,
                                          gpio_change_cb_, (void*)GPIO_INDEX_DC_SUPPLY_VALID_N);

/// GPIO info array for factorized handling with explicit index mapping
static gpio_info_t _gpio_infos[] = {
    [GPIO_INDEX_P3V3_PGOOD] = {p3V3_pgood_gpio_, gpio_states_.p3V3_pgood, "3V3 rail"},
    [GPIO_INDEX_P5V0_PGOOD] = {p5V0_pgood_gpio_, gpio_states_.p5V0_pgood, "5V0 rail"},
    [GPIO_INDEX_P7V5_PGOOD] = {p7V5_pgood_gpio_, gpio_states_.p7V5_pgood, "7V5 rail"},
    [GPIO_INDEX_PxVx_PGOOD] = {pxVx_pgood_gpio_, gpio_states_.pxVx_pgood, "Variable voltage rail"},
    [GPIO_INDEX_EMERGENCY_STOP] = {emergency_stop_gpio_, gpio_states_.emergency_stop,
                                   "Emergency stop"},
    [GPIO_INDEX_BATTERY_VALID_N] = {battery_valid_n_gpio_, gpio_states_.battery_valid,
                                    "Battery power source"},
    [GPIO_INDEX_DC_SUPPLY_VALID_N] = {dc_supply_valid_n_gpio_, gpio_states_.dc_supply_valid,
                                      "DC power source"},
};

/// @brief Number of GPIO infos in the array
/// @details Computed at compile time for array bounds checking and iteration
static constexpr size_t _gpio_infos_count = sizeof(_gpio_infos) / sizeof(_gpio_infos[0]);

/// @brief Display GPIO state with appropriate format
/// @param gpio_index Index of the GPIO to display
/// @param state Current state of the GPIO
/// @param name Name of the GPIO for display
static void display_gpio_state(gpio_index_t gpio_index, bool state, const char* name);

/// @brief Generic GPIO callback for all pin changes (called from ISR)
/// @param arg GPIO info index (gpio_index_t)
static void gpio_change_cb_(void* arg);

/// @brief GPIO handling thread function
/// @param args Unused thread arguments
/// @return nullptr (thread never exits)
/// @details This thread reads initial GPIO states at startup, then continuously
///          monitors for GPIO change messages from ISR callbacks and updates states accordingly
static void* _gpio_handling_thread(void* args);

void pf_init_power_supply(void) {}

void pf_init_power_supply_tasks(void)
{
    // Create GPIO handling thread
    _gpio_thread_pid = thread_create(
        _gpio_handling_thread_stack, sizeof(_gpio_handling_thread_stack), THREAD_PRIORITY_MAIN - 1,
        THREAD_CREATE_STACKTEST, _gpio_handling_thread, NULL, "GPIO handling thread");
}

void send_emergency_stop_status(void)
{
    cogip::canpb::CanProtobuf& canpb = pf_get_canpb();

    // Create protobuf message
    PB_EmergencyStopStatus pb_status;

    mutex_lock(&_gpio_states_mutex);
    pb_status.set_emergency_stop(gpio_states_.emergency_stop);
    mutex_unlock(&_gpio_states_mutex);

    // Send message
    if (!canpb.send_message(emergency_stop_status_uuid, &pb_status)) {
        LOG_ERROR("Error: emergency_stop_status_uuid message not sent\n");
    }
}

void send_power_source_status(void)
{
    cogip::canpb::CanProtobuf& canpb = pf_get_canpb();

    // Create protobuf message
    PB_PowerSourceStatus pb_status;

    mutex_lock(&_gpio_states_mutex);
    pb_status.set_battery_valid(gpio_states_.battery_valid);
    pb_status.set_dc_supply_valid(gpio_states_.dc_supply_valid);
    mutex_unlock(&_gpio_states_mutex);

    // Send message
    if (!canpb.send_message(power_source_status_uuid, &pb_status)) {
        LOG_ERROR("Error: power_source_status_uuid message not sent\n");
    }
}

void send_power_rails_status(void)
{
    cogip::canpb::CanProtobuf& canpb = pf_get_canpb();

    // Create protobuf message
    PB_PowerRailsStatus pb_status;

    mutex_lock(&_gpio_states_mutex);
    pb_status.set_p3V3_pgood(gpio_states_.p3V3_pgood);
    pb_status.set_p5V0_pgood(gpio_states_.p5V0_pgood);
    pb_status.set_p7V5_pgood(gpio_states_.p7V5_pgood);
    pb_status.set_pxVx_pgood(gpio_states_.pxVx_pgood);
    mutex_unlock(&_gpio_states_mutex);

    // Send message
    if (!canpb.send_message(power_rails_status_uuid, &pb_status)) {
        LOG_ERROR("Error: power_rails_status_uuid message not sent\n");
    }
}

static void display_gpio_state(gpio_index_t gpio_index, bool state, const char* name)
{
    switch (gpio_index) {
    case GPIO_INDEX_EMERGENCY_STOP:
        LOG_INFO("%s: %s\n", name, (state ? "released" : "engaged"));
        break;
    case GPIO_INDEX_BATTERY_VALID_N:
    case GPIO_INDEX_DC_SUPPLY_VALID_N:
        LOG_INFO("%s: %s\n", name, (state ? "disconnected" : "connected"));
        break;
    default:
        // Power rails GPIO
        LOG_INFO("%s: %s\n", name, (state ? "true" : "false"));
        break;
    }
}

static void gpio_change_cb_(void* arg)
{
    gpio_index_t gpio_index = static_cast<gpio_index_t>(reinterpret_cast<uintptr_t>(arg));

    // Check if GPIO thread is ready before sending message
    if (_gpio_thread_pid != KERNEL_PID_UNDEF) {
        // Send ISR-safe message to GPIO handling thread
        msg_t msg;
        msg.type = gpio_index;
        msg_send(&msg, _gpio_thread_pid);
    }
}

static void* _gpio_handling_thread([[maybe_unused]] void* args)
{
    msg_t msg;

    // Get initial state for all power supply GPIO at task startup to avoid missing any changes
    LOG_INFO("Power Supply Initial States:\n");
    for (size_t i = 0; i < _gpio_infos_count; i++) {
        bool new_state = _gpio_infos[i].gpio.read();

        mutex_lock(&_gpio_states_mutex);
        _gpio_infos[i].state_ref = new_state;
        mutex_unlock(&_gpio_states_mutex);

        display_gpio_state((gpio_index_t)i, new_state, _gpio_infos[i].name);
    }

    while (1) {
        // Wait for messages from GPIO ISR callbacks
        msg_receive(&msg);

        // Handle GPIO change based on message type (gpio_index_t)
        gpio_index_t gpio_index = (gpio_index_t)msg.type;

        if (gpio_index < _gpio_infos_count) {
            // Update the state by reading the GPIO with mutex protection
            bool new_state = _gpio_infos[gpio_index].gpio.read();

            mutex_lock(&_gpio_states_mutex);
            _gpio_infos[gpio_index].state_ref = new_state;
            mutex_unlock(&_gpio_states_mutex);

            // Sending can events based on GPIO type
            switch (gpio_index) {
            case GPIO_INDEX_BATTERY_VALID_N:
            case GPIO_INDEX_DC_SUPPLY_VALID_N:
                send_power_source_status();
                break;
            case GPIO_INDEX_EMERGENCY_STOP:
                send_emergency_stop_status();
                break;
            case GPIO_INDEX_P3V3_PGOOD:
            case GPIO_INDEX_P5V0_PGOOD:
            case GPIO_INDEX_P7V5_PGOOD:
            case GPIO_INDEX_PxVx_PGOOD:
                send_power_rails_status();
                break;
            default:
                break;
            }

            // Display the state change for this specific GPIO
            display_gpio_state(gpio_index, new_state, _gpio_infos[gpio_index].name);
        }
    }

    // Should never get here
    return nullptr;
}

} // namespace power_supply
} // namespace pf
} // namespace cogip

/// @}
