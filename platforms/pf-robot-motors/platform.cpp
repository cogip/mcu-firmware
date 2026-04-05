/* RIOT includes */
#include "log.h"

/* Project includes */
#include "canpb/ReadBuffer.hpp"
#include "pf_common/platform_common.hpp"
#include "platform.hpp"

/* Platform includes */
#include "pf_actuators.hpp"
#include "pf_parameters.hpp"

/// Start game message handler
static void _handle_game_start([[maybe_unused]] cogip::canpb::ReadBuffer& buffer)
{
    if (cogip::pf_common::is_emergency_stop_latched()) {
        LOG_WARNING("game_start rejected: emergency stop latched\n");
        return;
    }
    cogip::pf::actuators::enable_all();
}

/// Reset game message handler
static void _handle_game_reset([[maybe_unused]] cogip::canpb::ReadBuffer& buffer)
{
    cogip::pf_common::clear_emergency_stop();
    cogip::pf::actuators::enable_all();
}

/// End game message handler
static void _handle_game_end([[maybe_unused]] cogip::canpb::ReadBuffer& buffer)
{
    cogip::pf::actuators::disable_all();
}

/// Emergency stop callback
static void _on_emergency_stop()
{
    cogip::pf::actuators::disable_all();
}

void pf_init(void)
{
    /* Initialize common platform (CAN, heartbeat, copilot handlers) */
    int ret = cogip::pf_common::pf_init(
        {}, {}, cogip::pf_common::emergency_stop_callback_t::create<_on_emergency_stop>());
    if (ret) {
        LOG_ERROR("Common platform initialization failed, error: %d\n", ret);
    } else {
        // Register platform-specific message handlers
        cogip::canpb::CanProtobuf& canpb = cogip::pf_common::get_canpb();

        canpb.register_message_handler(
            game_reset_uuid, cogip::canpb::message_handler_t::create<_handle_game_reset>());
        canpb.register_message_handler(
            game_start_uuid, cogip::canpb::message_handler_t::create<_handle_game_start>());
        canpb.register_message_handler(game_end_uuid,
                                       cogip::canpb::message_handler_t::create<_handle_game_end>());
    }

    cogip::pf::actuators::init();
    cogip::pf::parameters::init();
}

void pf_init_tasks(void)
{
    cogip::pf_common::pf_init_tasks();
}
