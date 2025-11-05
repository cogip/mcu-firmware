/* RIOT includes */
#include "log.h"

/* Project includes */
#include "canpb/ReadBuffer.hpp"
#include "motion_control.hpp"
#include "pf_common/platform_common.hpp"
#include "platform.hpp"

/* Platform includes */
#include "trace_utils.hpp"

bool pf_trace_on(void)
{
    return cogip::pf_common::is_copilot_connected();
}

/// Start game message handler
static void _handle_game_start([[maybe_unused]] cogip::canpb::ReadBuffer& buffer)
{
    cogip::pf::motion_control::pf_enable_motion_control();
}

/// Reset game message handler
static void _handle_game_reset([[maybe_unused]] cogip::canpb::ReadBuffer& buffer)
{
    cogip::pf::motion_control::pf_disable_motion_control();

    cogip::pf::motion_control::pf_motion_control_reset();
}

/// Start threading sending actuators state.
static void _handle_game_end([[maybe_unused]] cogip::canpb::ReadBuffer& buffer)
{
    cogip::pf::motion_control::pf_handle_game_end(buffer);
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
        canpb.register_message_handler(
            brake_uuid,
            cogip::canpb::message_handler_t::create<cogip::pf::motion_control::pf_handle_brake>());
        canpb.register_message_handler(pose_order_uuid,
                                       cogip::canpb::message_handler_t::create<
                                           cogip::pf::motion_control::pf_handle_target_pose>());
        canpb.register_message_handler(pose_start_uuid,
                                       cogip::canpb::message_handler_t::create<
                                           cogip::pf::motion_control::pf_handle_start_pose>());

        canpb.send_message(reset_uuid);
    }

    cogip::pf::motion_control::pf_init_motion_control();
}

void pf_init_tasks(void)
{
    cogip::pf_common::pf_init_tasks();

    trace_start();

    cogip::pf::motion_control::pf_start_motion_control();
}
