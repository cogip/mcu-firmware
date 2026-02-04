// RIOT includes
#include "log.h"
#include "periph/qdec.h"
#include <inttypes.h>

#define ENABLE_DEBUG 0
#include <debug.h>

// Project includes
#include "app.hpp"
#include "app_conf.hpp"
#include "board.h"
#include "drive_controller/DifferentialDriveController.hpp"
#include "drive_controller/DifferentialDriveControllerParameters.hpp"
#include "encoder/EncoderQDEC.hpp"
#include "localization/LocalizationDifferential.hpp"
#include "motion_control.hpp"
#include "motion_control_common/MetaController.hpp"
#include "motion_motors_params.hpp"
#include "motor/MotorDriverDRV8873.hpp"
#include "motor/MotorRIOT.hpp"
#include "path/Path.hpp"
#include "path/Pose.hpp"
#include "platform.hpp"
#include "platform_engine/PlatformEngine.hpp"

#include "adaptive_pure_pursuit_chain.hpp"
#include "angular_pose_tuning_chain.hpp"
#include "angular_speed_tuning_chain.hpp"
#include "linear_pose_tuning_chain.hpp"
#include "linear_speed_tuning_chain.hpp"
#include "pose_test_chain.hpp"
#include "quadpid_chain.hpp"
#include "quadpid_tracker_chain.hpp"

#include "PB_Controller.hpp"
#include "PB_PathPose.hpp"
#include "PB_State.hpp"
#include "telemetry/Telemetry.hpp"

namespace cogip {

namespace pf {

namespace motion_control {

using cogip::utils::operator"" _key_hash;

// Current controller
static uint32_t current_controller_id = 0;

// Protobuf
PB_Pose pb_pose;
PB_Controller pb_controller;
PB_State pb_state;

// PID tuning period
constexpr uint16_t motion_control_pid_tuning_period_ms = 1500;

// Target pose
static cogip::path::Pose target_pose;
// Target speed
static cogip::cogip_defs::Polar target_speed;

/// Encoders
static cogip::encoder::EncoderQDEC left_encoder(MOTOR_LEFT, COGIP_BOARD_ENCODER_MODE,
                                                encoder_wheels_resolution_pulses.get());
static cogip::encoder::EncoderQDEC right_encoder(MOTOR_RIGHT, COGIP_BOARD_ENCODER_MODE,
                                                 encoder_wheels_resolution_pulses.get());

/// Odometry
static cogip::localization::LocalizationDifferentialParameters
    localization_params(left_encoder_wheels_diameter_mm, right_encoder_wheels_diameter_mm,
                        encoder_wheels_distance_mm, qdec_left_polarity, qdec_right_polarity);
static cogip::localization::LocalizationDifferential localization(localization_params, left_encoder,
                                                                  right_encoder);

/// Motor driver
static cogip::motor::MotorDriverDRV8873 motor_driver(motion_motors_params);

/// Motors
static cogip::motor::MotorRIOT left_motor(motor_driver, MOTOR_LEFT);
static cogip::motor::MotorRIOT right_motor(motor_driver, MOTOR_RIGHT);

static cogip::drive_controller::DifferentialDriveControllerParameters
    drive_controller_params(motor_wheels_diameter_mm, motor_wheels_diameter_mm,
                            motor_wheels_distance_mm, left_motor_constant, right_motor_constant,
                            min_motor_speed_percent, max_motor_speed_percent,
                            motion_control_thread_period_ms);

static cogip::drive_controller::DifferentialDriveController
    drive_controller(drive_controller_params, left_motor, right_motor);
static void pf_pose_reached_cb(const cogip::motion_control::target_pose_status_t state);
// Motion control engine
static cogip::motion_control::PlatformEngine pf_motion_control_platform_engine(
    localization, drive_controller,
    cogip::motion_control::pose_reached_cb_t::create<pf_pose_reached_cb>(),
    motion_control_thread_period_ms);

/// Handle controller change request
static void _handle_set_controller(cogip::canpb::ReadBuffer& buffer)
{
    pb_controller.clear();

    EmbeddedProto::Error error = pb_controller.deserialize(buffer);
    if (error != EmbeddedProto::Error::NO_ERRORS) {
        LOG_ERROR("Controller change request: Protobuf deserialization error: %d\n",
                  static_cast<int>(error));
        return;
    }

    // Change controller
    current_controller_id = static_cast<uint32_t>(pb_controller.id());
    switch (static_cast<uint32_t>(pb_controller.id())) {
    case static_cast<uint32_t>(PB_ControllerEnum::QUADPID_TRACKER):
        LOG_INFO("Change to controller: QUADPID_TRACKER\n");
        pf_motion_control_platform_engine.set_controller(
            &quadpid_tracker_chain::quadpid_tracker_meta_controller);
        pf_motion_control_platform_engine.set_timeout_enable(false);
        break;

    case static_cast<uint32_t>(PB_ControllerEnum::LINEAR_SPEED_TUNING):
        LOG_INFO("Change to controller: LINEAR_SPEED_TUNING\n");
        pf_motion_control_platform_engine.set_controller(
            &linear_speed_tuning_chain::meta_controller);
        pf_motion_control_platform_engine.set_timeout_enable(true);
        break;

    case static_cast<uint32_t>(PB_ControllerEnum::ANGULAR_SPEED_TUNING):
        LOG_INFO("Change to controller: ANGULAR_SPEED_TUNING\n");
        pf_motion_control_platform_engine.set_controller(
            &angular_speed_tuning_chain::meta_controller);
        pf_motion_control_platform_engine.set_timeout_enable(true);
        break;

    case static_cast<uint32_t>(PB_ControllerEnum::LINEAR_POSE_TUNING):
        LOG_INFO("Change to controller: LINEAR_POSE_TUNING\n");
        pf_motion_control_platform_engine.set_controller(
            &linear_pose_tuning_chain::meta_controller);
        pf_motion_control_platform_engine.set_timeout_enable(true);
        break;

    case static_cast<uint32_t>(PB_ControllerEnum::ANGULAR_POSE_TUNING):
        LOG_INFO("Change to controller: ANGULAR_POSE_TUNING\n");
        pf_motion_control_platform_engine.set_controller(
            &angular_pose_tuning_chain::meta_controller);
        pf_motion_control_platform_engine.set_timeout_enable(true);
        break;

    case static_cast<uint32_t>(PB_ControllerEnum::LINEAR_POSE_TEST):
        LOG_INFO("Change to controller: LINEAR_POSE_TEST\n");
        pf_motion_control_platform_engine.set_controller(&pose_test_chain::meta_controller);
        pf_motion_control_platform_engine.set_timeout_enable(true);
        break;

    case static_cast<uint32_t>(PB_ControllerEnum::ADAPTIVE_PURE_PURSUIT):
        LOG_INFO("Change to controller: ADAPTIVE_PURE_PURSUIT\n");
        pf_motion_control_platform_engine.set_controller(
            &adaptive_pure_pursuit_chain::meta_controller);
        pf_motion_control_platform_engine.set_timeout_enable(false);
        break;

    case static_cast<uint32_t>(PB_ControllerEnum::QUADPID):
    default:
        LOG_INFO("Change to controller: QUADPID\n");
        pf_motion_control_platform_engine.set_controller(&quadpid_chain::quadpid_meta_controller);
        pf_motion_control_platform_engine.set_timeout_enable(false);
        break;
    }

    pf_motion_control_platform_engine.set_current_cycle(0);
    pf_motion_control_platform_engine.dump_pipeline();
}

static void pf_pose_reached_cb(const cogip::motion_control::target_pose_status_t state)
{
    // Previous target pose status flag to avoid flooding protobuf can bus.
    static cogip::motion_control::target_pose_status_t previous_target_pose_status =
        cogip::motion_control::target_pose_status_t::moving;

    switch (state) {
    case cogip::motion_control::target_pose_status_t::moving:
        // Do nothing when robot is moving
        break;

    case cogip::motion_control::target_pose_status_t::reached:
        // Send message when pose is reached
        // Note: Path advancement is handled by PathManagerFilter in the control loop.
        // This callback only sends CAN messages.
        if (previous_target_pose_status != state) {
            // Read is_intermediate from IO (set by PathManagerFilter, PurePursuit, or
            // PlatformEngine)
            bool is_intermediate = false;
            if (auto opt = pf_motion_control_platform_engine.io().get_as<bool>("is_intermediate")) {
                is_intermediate = *opt;
            }

            if (is_intermediate) {
                pf_get_canpb().send_message(intermediate_pose_reached_uuid);
            } else {
                pf_get_canpb().send_message(pose_reached_uuid);

                // Read path_complete from IO (set by PathManagerFilter or PurePursuit)
                if (auto opt =
                        pf_motion_control_platform_engine.io().get_as<bool>("path_complete")) {
                    if (*opt) {
                        pf_get_canpb().send_message(path_complete_uuid);
                    }
                }
            }
        }
        break;

    case cogip::motion_control::target_pose_status_t::blocked:
        if (pf_motion_control_platform_engine.target_pose().bypass_anti_blocking()) {
            // Set current robot pose as target pose to avoid robot moving
            target_pose.set_x(pf_motion_control_platform_engine.current_pose().x());
            target_pose.set_y(pf_motion_control_platform_engine.current_pose().y());
            target_pose.set_O(pf_motion_control_platform_engine.current_pose().O());
            pf_motion_control_platform_engine.set_target_pose(target_pose);

            // Consider pose_reached as anti blocking is bypassed
            pf_motion_control_platform_engine.set_pose_reached(
                cogip::motion_control::target_pose_status_t::reached);

            // As pose is reached, pose straight filter state machine is in finished
            // state (reset both chains as only one is active at a time)
            quadpid_chain::pose_straight_filter.force_finished_state();
            quadpid_tracker_chain::pose_straight_filter.force_finished_state();

            LOG_WARNING("BLOCKED bypassed\n");

            pf_get_canpb().send_message(pose_reached_uuid);
        } else {
            // Stop motors
            left_motor.set_speed(0);
            right_motor.set_speed(0);

            // As motors are stopped, pose straight filter state machine is in
            // finished state (reset both chains as only one is active at a time)
            quadpid_chain::pose_straight_filter.force_finished_state();
            quadpid_tracker_chain::pose_straight_filter.force_finished_state();

            LOG_WARNING("BLOCKED\n");

            if (previous_target_pose_status !=
                cogip::motion_control::target_pose_status_t::blocked) {
                pf_get_canpb().send_message(blocked_uuid);
            }
        }

        break;

    default:
        break;
    }

    // Reset filters and PIDs if state changed and is not moving
    if (previous_target_pose_status != state &&
        state != cogip::motion_control::target_pose_status_t::moving) {
        switch (current_controller_id) {
        case static_cast<uint32_t>(PB_ControllerEnum::QUADPID):
            quadpid_chain::reset();
            break;
        case static_cast<uint32_t>(PB_ControllerEnum::QUADPID_TRACKER):
            quadpid_tracker_chain::reset();
            break;
        default:
            // Other chains don't have stateful filters to reset
            break;
        }
    }

    // Backup target pose status flag to avoid flooding protobuf serial bus on
    // next cycle.
    previous_target_pose_status = state;
}

/*
 * Used for simulation in motion_motors_params.hpp only if MOTION_MOTORS_POST_CB
 * is defined in board.h this way:
 * #define MOTION_MOTORS_POST_CB cogip_native_motor_driver_qdec_simulation
 */
extern "C" {
extern int32_t qdecs_value[QDEC_NUMOF];
}

void cogip_native_motor_driver_qdec_simulation(const motor_driver_t* motor_driver, uint8_t motor_id,
                                               int32_t pwm_duty_cycle)
{
    // Unused variables
    (void)motor_driver;
    (void)motor_id;
    (void)pwm_duty_cycle;

    float wheels_perimeter = M_PI * left_encoder_wheels_diameter_mm.get();
    float pulse_per_mm = encoder_wheels_resolution_pulses.get() / wheels_perimeter;
    float wheels_distance_pulse = encoder_wheels_distance_mm.get() * pulse_per_mm;
    float pulse_per_degree = (wheels_distance_pulse * 2 * M_PI) / 360;

    // On native architecture set speeds at their theorical value, no error.
    if (pf_motion_control_platform_engine.pose_reached() !=
        cogip::motion_control::target_pose_status_t::reached) {
        // Get speed commands from IO
        float linear_speed_cmd = 0.0f;
        float angular_speed_cmd = 0.0f;
        if (auto opt = pf_motion_control_platform_engine.io().get_as<float>("linear_speed_order")) {
            linear_speed_cmd = *opt;
        }
        if (auto opt =
                pf_motion_control_platform_engine.io().get_as<float>("angular_speed_order")) {
            angular_speed_cmd = *opt;
        }

        qdecs_value[MOTOR_RIGHT] =
            (linear_speed_cmd * pulse_per_mm + angular_speed_cmd * pulse_per_degree / 2) *
            qdec_right_polarity.get();
        qdecs_value[MOTOR_LEFT] =
            (linear_speed_cmd * pulse_per_mm - angular_speed_cmd * pulse_per_degree / 2) *
            qdec_left_polarity.get();
    }
}

void pf_send_pb_pose(void)
{
    cogip_defs::Pose current_pose = pf_motion_control_platform_engine.current_pose();

    // Copy current pose
    current_pose.pb_copy(pb_pose);

    pf_get_canpb().send_message(pose_order_uuid, &pb_pose);
}

void pf_send_pb_state(void)
{
    pb_state.clear();
    pb_state.mutable_cycle() = pf_motion_control_platform_engine.current_cycle();
    pf_motion_control_platform_engine.current_speed().pb_copy(pb_state.mutable_speed_current());
    pf_motion_control_platform_engine.target_speed().pb_copy(pb_state.mutable_speed_order());

    pf_get_canpb().send_message(state_uuid, &pb_state);
}

void pf_send_encoder_telemetry(void)
{
    cogip::telemetry::Telemetry::send<int64_t>("encoder_left"_key_hash, left_encoder.counter());
    cogip::telemetry::Telemetry::send<int64_t>("encoder_right"_key_hash, right_encoder.counter());
}

void pf_handle_brake([[maybe_unused]] cogip::canpb::ReadBuffer& buffer)
{
    pf_motion_control_platform_engine.set_target_speed(cogip::cogip_defs::Polar(0, 0));

    // Reset both chains as only one is active at a time
    quadpid_chain::pose_straight_filter.force_finished_state();
    quadpid_tracker_chain::pose_straight_filter.force_finished_state();
}

void pf_handle_game_end([[maybe_unused]] cogip::canpb::ReadBuffer& buffer)
{
    pf_motion_control_platform_engine.set_target_speed(cogip::cogip_defs::Polar(0, 0));

    // Reset filters according to current controller
    switch (current_controller_id) {
    case static_cast<uint32_t>(PB_ControllerEnum::QUADPID):
        quadpid_chain::reset();
        break;
    case static_cast<uint32_t>(PB_ControllerEnum::QUADPID_TRACKER):
        quadpid_tracker_chain::reset();
        break;
    default:
        break;
    }

    // Force position filter finished state (reset both chains as only one is active at a time)
    quadpid_chain::pose_straight_filter.force_finished_state();
    quadpid_tracker_chain::pose_straight_filter.force_finished_state();

    // Disable motion control to avoid new motion
    pf_disable_motion_control();
}

void pf_handle_target_pose(cogip::canpb::ReadBuffer& buffer)
{
    // Retrieve new target pose from protobuf message
    PB_PathPose pb_path_target_pose;
    EmbeddedProto::Error error = pb_path_target_pose.deserialize(buffer);
    if (error != EmbeddedProto::Error::NO_ERRORS) {
        DEBUG("Pose to reach: Protobuf deserialization error: %d\n", static_cast<int>(error));
        return;
    }

    cogip::path::Pose previous_target_pose = target_pose;

    // Target pose
    target_pose.pb_read(pb_path_target_pose);
    LOG_INFO("New target pose: x=%.2f, y=%.2f, O=%.2f\n", static_cast<double>(target_pose.x()),
             static_cast<double>(target_pose.y()), static_cast<double>(target_pose.O()));

    // Use path manager for backward compatibility: reset + add + start
    cogip::path::Path::instance().reset();
    cogip::path::Path::instance().add_point(target_pose);
    cogip::path::Path::instance().start();

    // Target speed
    target_speed.set_distance(
        (platform_max_speed_linear_mm_per_period * target_pose.max_speed_ratio_linear()) / 100);
    target_speed.set_angle(
        (platform_max_speed_angular_deg_per_period * target_pose.max_speed_ratio_angular()) / 100);
    pf_motion_control_platform_engine.set_target_speed(target_speed);
    LOG_INFO("Target speed: linear=%.2f mm/period, angular=%.2f deg/period\n",
             static_cast<double>(target_speed.distance()),
             static_cast<double>(target_speed.angle()));

    // Set target speed for passthrough controllers
    quadpid_chain::passthrough_linear_pose_controller_parameters.set_target_speed(
        target_speed.distance());
    quadpid_chain::passthrough_angular_pose_controller_parameters.set_target_speed(
        target_speed.angle());

    if (target_pose.timeout_ms()) {
        pf_motion_control_platform_engine.set_timeout_enable(true);
        pf_motion_control_platform_engine.set_timeout_ms(target_pose.timeout_ms());
    } else {
        pf_motion_control_platform_engine.set_timeout_enable(false);
    }

    // Deal with the first pose in the list
    pf_motion_control_platform_engine.set_target_pose(target_pose);

    pf_motion_control_platform_engine.set_pose_reached(
        cogip::motion_control::target_pose_status_t::moving);

    pf_motion_control_platform_engine.disable();
    pf_motion_control_reset_controllers();
    pf_motion_control_platform_engine.enable();
}

void pf_handle_start_pose(cogip::canpb::ReadBuffer& buffer)
{
    PB_PathPose pb_start_pose;
    EmbeddedProto::Error error = pb_start_pose.deserialize(buffer);
    if (error != EmbeddedProto::Error::NO_ERRORS) {
        LOG_ERROR("Current pose: Protobuf deserialization error: %d\n", static_cast<int>(error));
        return;
    }
    cogip::path::Pose start_pose;
    start_pose.pb_read(pb_start_pose);
    cogip::path::Pose pose(start_pose.x(), start_pose.y(), start_pose.O());

    LOG_INFO("[START_POSE] Received from planner: x=%.1f y=%.1f O=%.1f\n",
             static_cast<double>(pose.x()), static_cast<double>(pose.y()),
             static_cast<double>(pose.O()));

    pf_motion_control_platform_engine.set_current_pose(pose);
    pf_motion_control_platform_engine.set_target_pose(pose);
    // New start pose, the robot is not moving
    pf_motion_control_platform_engine.set_pose_reached(
        cogip::motion_control::target_pose_status_t::reached);

    pf_motion_control_platform_engine.set_current_cycle(0);

    LOG_INFO("[START_POSE] Localization updated and robot marked as stationary\n");
}

void pf_handle_path_reset([[maybe_unused]] cogip::canpb::ReadBuffer& buffer)
{
    LOG_INFO("[PATH_RESET] Clearing path\n");
    cogip::path::Path::instance().reset();
}

void pf_handle_path_add_point(cogip::canpb::ReadBuffer& buffer)
{
    // Retrieve pose from protobuf message
    PB_PathPose pb_path_pose;
    EmbeddedProto::Error error = pb_path_pose.deserialize(buffer);
    if (error != EmbeddedProto::Error::NO_ERRORS) {
        LOG_ERROR("[PATH_ADD_POINT] Protobuf deserialization error: %d\n", static_cast<int>(error));
        return;
    }

    if (cogip::path::Path::instance().add_point_from_pb(pb_path_pose)) {
        const auto* added =
            cogip::path::Path::instance().waypoint_at(cogip::path::Path::instance().size() - 1);
        if (added) {
            LOG_INFO("[PATH_ADD_POINT] Added waypoint %u: x=%.1f, y=%.1f, O=%.1f\n",
                     static_cast<unsigned>(cogip::path::Path::instance().size()),
                     static_cast<double>(added->x()), static_cast<double>(added->y()),
                     static_cast<double>(added->O()));
        }
    } else {
        LOG_ERROR("[PATH_ADD_POINT] Path full, cannot add waypoint\n");
    }
}

void pf_handle_path_start([[maybe_unused]] cogip::canpb::ReadBuffer& buffer)
{
    LOG_INFO("[PATH_START] Starting path execution with %u waypoints\n",
             static_cast<unsigned>(cogip::path::Path::instance().size()));

    if (cogip::path::Path::instance().empty()) {
        LOG_WARNING("[PATH_START] Path is empty, nothing to do\n");
        return;
    }

    cogip::path::Path::instance().start();

    // Get first target pose from path
    const cogip::path::Pose* first_pose = cogip::path::Path::instance().current_pose();
    if (first_pose) {
        target_pose = *first_pose;

        // Target speed
        target_speed.set_distance(
            (platform_max_speed_linear_mm_per_period * target_pose.max_speed_ratio_linear()) / 100);
        target_speed.set_angle(
            (platform_max_speed_angular_deg_per_period * target_pose.max_speed_ratio_angular()) /
            100);
        pf_motion_control_platform_engine.set_target_speed(target_speed);

        // Set target speed for passthrough controllers
        quadpid_chain::passthrough_linear_pose_controller_parameters.set_target_speed(
            target_speed.distance());
        quadpid_chain::passthrough_angular_pose_controller_parameters.set_target_speed(
            target_speed.angle());

        if (target_pose.timeout_ms()) {
            pf_motion_control_platform_engine.set_timeout_enable(true);
            pf_motion_control_platform_engine.set_timeout_ms(target_pose.timeout_ms());
        } else {
            pf_motion_control_platform_engine.set_timeout_enable(false);
        }

        // Set target pose on engine
        pf_motion_control_platform_engine.set_target_pose(target_pose);

        // Reset pose_reached to moving (also clears IO to avoid stale signals)
        pf_motion_control_platform_engine.reset_pose_reached();

        // Reset controllers for new path
        pf_motion_control_reset_controllers();

        // Ensure engine is enabled
        pf_motion_control_platform_engine.enable();
    }
}

void pf_start_motion_control(void)
{
    // Start engine thread
    pf_motion_control_platform_engine.start_thread();
}

void pf_motion_control_reset_controllers(void)
{
    // Log pose before reset
    const auto& pose_before = localization.pose();
    LOG_INFO("[RESET] Pose BEFORE reset: x=%.1f y=%.1f O=%.1f\n",
             static_cast<double>(pose_before.x()), static_cast<double>(pose_before.y()),
             static_cast<double>(pose_before.O()));

    // Reset filters according to current controller
    // Note: chain::reset() resets PIDs and filters but NOT PoseStraightFilter state machine
    // The state machine must manage its own transitions based on pose errors
    switch (current_controller_id) {
    case static_cast<uint32_t>(PB_ControllerEnum::QUADPID):
        quadpid_chain::reset();
        break;
    case static_cast<uint32_t>(PB_ControllerEnum::QUADPID_TRACKER):
        quadpid_tracker_chain::reset();
        break;
    case static_cast<uint32_t>(PB_ControllerEnum::ADAPTIVE_PURE_PURSUIT):
        adaptive_pure_pursuit_chain::reset();
        break;
    default:
        break;
    }

    // Log pose after reset
    const auto& pose_after = localization.pose();
    LOG_INFO("[RESET] Pose AFTER reset: x=%.1f y=%.1f O=%.1f\n",
             static_cast<double>(pose_after.x()), static_cast<double>(pose_after.y()),
             static_cast<double>(pose_after.O()));
}

static void pf_encoder_reset(void)
{
    left_encoder.reset();
    right_encoder.reset();
}

void pf_disable_motion_control()
{
    LOG_INFO("[DISABLE] Disabling motion control and resetting encoders\n");
    pf_motion_control_platform_engine.disable();

    // Disable motors
    left_motor.disable();
    right_motor.disable();

    pf_encoder_reset();
    LOG_INFO("[DISABLE] Encoders reset to zero\n");
}

void pf_enable_motion_control()
{
    pf_motion_control_platform_engine.enable();
}

void pf_init_motion_control(void)
{
    // Init motors
    left_motor.init();
    right_motor.init();

    // Init encoders
    int error = left_encoder.init();
    if (error) {
        LOG_ERROR("QDEC %" PRIu32 " not initialized, error=%d\n", static_cast<uint32_t>(MOTOR_LEFT),
                  error);
    }
    error = right_encoder.init();
    if (error) {
        LOG_ERROR("QDEC %" PRIu32 " not initialized, error=%d\n",
                  static_cast<uint32_t>(MOTOR_RIGHT), error);
    }

    // Init controllers
    quadpid_chain::init();
    quadpid_tracker_chain::init();
    adaptive_pure_pursuit_chain::init();
    linear_speed_tuning_chain::init();
    angular_speed_tuning_chain::init();
    linear_pose_tuning_chain::init();
    angular_pose_tuning_chain::init();
    pose_test_chain::init();

    // Associate default controller (QUADPID_TRACKER) to the engine
    pf_motion_control_platform_engine.set_controller(
        &quadpid_tracker_chain::quadpid_tracker_meta_controller);
    pf_motion_control_platform_engine.dump_pipeline();

    // Set timeout for speed only loops as no pose has to be reached
    pf_motion_control_platform_engine.set_timeout_ms(motion_control_pid_tuning_period_ms /
                                                     motion_control_thread_period_ms);

    // Register new pids config
    pf_get_canpb().register_message_handler(
        controller_uuid, cogip::canpb::message_handler_t::create<_handle_set_controller>());

    pf_encoder_reset();
    pf_disable_motion_control();
}

} // namespace motion_control

} // namespace pf

} // namespace cogip
