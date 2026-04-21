/* RIOT includes */
#include "log.h"

/* Project includes */
#include "canpb/ReadBuffer.hpp"
#include "motion_control.hpp"
#include "motion_control_parameters.hpp"
#include "pf_common/platform_common.hpp"
#include "platform.hpp"
#include "telemetry/Telemetry.hpp"

/* Platform includes */
#include "trace_utils.hpp"

static void _handle_game_start([[maybe_unused]] cogip::canpb::ReadBuffer& buffer);
static void _handle_game_reset([[maybe_unused]] cogip::canpb::ReadBuffer& buffer);
static void _handle_game_end([[maybe_unused]] cogip::canpb::ReadBuffer& buffer);
static void _handle_brake([[maybe_unused]] cogip::canpb::ReadBuffer& buffer);
static void _handle_pose_order([[maybe_unused]] cogip::canpb::ReadBuffer& buffer);
static void _handle_speed_order([[maybe_unused]] cogip::canpb::ReadBuffer& buffer);
static void _handle_pose_start([[maybe_unused]] cogip::canpb::ReadBuffer& buffer);
static void _handle_path_reset([[maybe_unused]] cogip::canpb::ReadBuffer& buffer);
static void _handle_path_add_point([[maybe_unused]] cogip::canpb::ReadBuffer& buffer);
static void _handle_path_start([[maybe_unused]] cogip::canpb::ReadBuffer& buffer);
static void _handle_parameter_get([[maybe_unused]] cogip::canpb::ReadBuffer& buffer);
static void _handle_parameter_set([[maybe_unused]] cogip::canpb::ReadBuffer& buffer);
static void _handle_parameter_announce([[maybe_unused]] cogip::canpb::ReadBuffer& buffer);
static void _on_copilot_connected([[maybe_unused]] cogip::canpb::ReadBuffer& buffer);
static void _handle_telemetry_enable([[maybe_unused]] cogip::canpb::ReadBuffer& buffer);
static void _handle_telemetry_disable([[maybe_unused]] cogip::canpb::ReadBuffer& buffer);
static void _on_emergency_stop();

bool pf_trace_on(void)
{
    return cogip::pf_common::is_copilot_connected();
}

void pf_init(void)
{
    /* Initialize common platform (CAN, heartbeat, copilot handlers) */
    int ret = cogip::pf_common::pf_init(
        cogip::pf_common::copilot_callback_t::create<_on_copilot_connected>(),
        {},
        cogip::pf_common::emergency_stop_callback_t::create<_on_emergency_stop>());
    if (ret) {
        LOG_ERROR("Common platform initialization failed, error: %d\n", ret);
    } else {
        // Register platform-specific message handlers
        cogip::canpb::CanProtobuf& canpb = cogip::pf_common::get_canpb();

        // clang-format off
        canpb.register_message_handler(game_reset_uuid,
                                       cogip::canpb::message_handler_t::create<_handle_game_reset>());
        canpb.register_message_handler(game_start_uuid,
                                       cogip::canpb::message_handler_t::create<_handle_game_start>());
        canpb.register_message_handler(game_end_uuid,
                                       cogip::canpb::message_handler_t::create<_handle_game_end>());
        canpb.register_message_handler(brake_uuid,
                                       cogip::canpb::message_handler_t::create<_handle_brake>());
        canpb.register_message_handler(pose_order_uuid,
                                       cogip::canpb::message_handler_t::create<_handle_pose_order>());
        canpb.register_message_handler(speed_order_uuid,
                                       cogip::canpb::message_handler_t::create<_handle_speed_order>());
        canpb.register_message_handler(pose_start_uuid,
                                       cogip::canpb::message_handler_t::create<_handle_pose_start>());
        canpb.register_message_handler(path_reset_uuid,
                                       cogip::canpb::message_handler_t::create<_handle_path_reset>());
        canpb.register_message_handler(path_add_point_uuid,
                                       cogip::canpb::message_handler_t::create<_handle_path_add_point>());
        canpb.register_message_handler(path_start_uuid,
                                       cogip::canpb::message_handler_t::create<_handle_path_start>());
        canpb.register_message_handler(parameter_get_uuid,
                                       cogip::canpb::message_handler_t::create<_handle_parameter_get>());
        canpb.register_message_handler(parameter_set_uuid,
                                       cogip::canpb::message_handler_t::create<_handle_parameter_set>());
        canpb.register_message_handler(parameter_announce_request_uuid,
                                       cogip::canpb::message_handler_t::create<_handle_parameter_announce>());
        canpb.register_message_handler(telemetry_enable_uuid,
                                       cogip::canpb::message_handler_t::create<_handle_telemetry_enable>());
        canpb.register_message_handler(telemetry_disable_uuid,
                                       cogip::canpb::message_handler_t::create<_handle_telemetry_disable>());
        // clang-format on

        // Initialize telemetry
        cogip::telemetry::Telemetry::init(canpb, telemetry_data_uuid);

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

/// Start game message handler
static void _handle_game_start([[maybe_unused]] cogip::canpb::ReadBuffer& buffer)
{
    if (cogip::pf_common::is_emergency_stop_latched()) {
        LOG_WARNING("game_start rejected: emergency stop latched\n");
        return;
    }
    cogip::pf::motion_control::pf_enable_motion_control();
}

/// Reset game message handler
static void _handle_game_reset([[maybe_unused]] cogip::canpb::ReadBuffer& buffer)
{
    cogip::pf_common::clear_emergency_stop();
    cogip::pf::motion_control::pf_disable_motion_control();

    cogip::pf::motion_control::pf_motion_control_reset_controllers();
}

/// End game message handler
static void _handle_game_end([[maybe_unused]] cogip::canpb::ReadBuffer& buffer)
{
    cogip::pf::motion_control::pf_handle_game_end(buffer);
}

/// Brake message handler
static void _handle_brake([[maybe_unused]] cogip::canpb::ReadBuffer& buffer)
{
    cogip::pf::motion_control::pf_handle_brake(buffer);
}

/// Pose order message handler
static void _handle_pose_order([[maybe_unused]] cogip::canpb::ReadBuffer& buffer)
{
    if (cogip::pf_common::is_emergency_stop_latched()) {
        LOG_WARNING("pose_order rejected: emergency stop latched\n");
        return;
    }
    cogip::pf::motion_control::pf_handle_target_pose(buffer);
}

/// Speed order message handler (speed PID tuning)
static void _handle_speed_order([[maybe_unused]] cogip::canpb::ReadBuffer& buffer)
{
    if (cogip::pf_common::is_emergency_stop_latched()) {
        LOG_WARNING("speed_order rejected: emergency stop latched\n");
        return;
    }
    cogip::pf::motion_control::pf_handle_speed_order(buffer);
}

/// Pose start message handler
static void _handle_pose_start([[maybe_unused]] cogip::canpb::ReadBuffer& buffer)
{
    if (cogip::pf_common::is_emergency_stop_latched()) {
        LOG_WARNING("pose_start rejected: emergency stop latched\n");
        return;
    }
    cogip::pf::motion_control::pf_handle_start_pose(buffer);
}

/// Path reset message handler
static void _handle_path_reset([[maybe_unused]] cogip::canpb::ReadBuffer& buffer)
{
    cogip::pf::motion_control::pf_handle_path_reset(buffer);
}

/// Path add point message handler
static void _handle_path_add_point([[maybe_unused]] cogip::canpb::ReadBuffer& buffer)
{
    cogip::pf::motion_control::pf_handle_path_add_point(buffer);
}

/// Path start message handler
static void _handle_path_start([[maybe_unused]] cogip::canpb::ReadBuffer& buffer)
{
    if (cogip::pf_common::is_emergency_stop_latched()) {
        LOG_WARNING("path_start rejected: emergency stop latched\n");
        return;
    }
    cogip::pf::motion_control::pf_handle_path_start(buffer);
}

/// Parameter get message handler
static void _handle_parameter_get([[maybe_unused]] cogip::canpb::ReadBuffer& buffer)
{
    cogip::pf::motion_control::pf_handle_parameter_get(buffer);
}

/// Parameter set message handler
static void _handle_parameter_set([[maybe_unused]] cogip::canpb::ReadBuffer& buffer)
{
    cogip::pf::motion_control::pf_handle_parameter_set(buffer);
}

/// Parameter announce message handler
static void _handle_parameter_announce([[maybe_unused]] cogip::canpb::ReadBuffer& buffer)
{
    cogip::pf::motion_control::pf_handle_parameter_announce(buffer);
}

/// Telemetry enable message handler
static void _handle_telemetry_enable([[maybe_unused]] cogip::canpb::ReadBuffer& buffer)
{
    cogip::telemetry::Telemetry::enable();
}

/// Telemetry disable message handler
static void _handle_telemetry_disable([[maybe_unused]] cogip::canpb::ReadBuffer& buffer)
{
    cogip::telemetry::Telemetry::disable();
}

/// Copilot-connected callback: pushes every registered parameter upstream so
/// the host can populate its cache without having to send an announce
/// request at startup. The request-driven path
/// (parameter_announce_request_uuid) remains available for targeted queries.
static void _on_copilot_connected([[maybe_unused]] cogip::canpb::ReadBuffer& buffer)
{
    cogip::pf::motion_control::pf_auto_announce_parameters();
}

/// Emergency stop callback
static void _on_emergency_stop()
{
    cogip::pf::motion_control::pf_disable_motion_control();
    cogip::pf::motion_control::pf_motion_control_reset_controllers();
}
