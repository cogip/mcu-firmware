/* RIOT includes */
#include "log.h"

/* Project includes */
#include "canpb/ReadBuffer.hpp"
#include "pf_common/platform_common.hpp"
#include "platform.hpp"

/* Platform includes */
#include "pf_actuators.hpp"

/// Start game message handler
static void _handle_game_start([[maybe_unused]] cogip::canpb::ReadBuffer& buffer)
{
    cogip::pf::actuators::enable_all();
}

/// Reset game message handler
static void _handle_game_reset([[maybe_unused]] cogip::canpb::ReadBuffer& buffer)
{
    cogip::pf::actuators::enable_all();
}

/// Start threading sending actuators state.
static void _handle_game_end([[maybe_unused]] cogip::canpb::ReadBuffer& buffer)
{
    cogip::pf::actuators::disable_all();
}

void pf_init(void)
{
    /* Initialize common platform (CAN, heartbeat, copilot handlers) */
    int ret = cogip::pf_common::pf_init();
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
}

void pf_init_tasks(void)
{
    cogip::pf_common::pf_init_tasks();
}
