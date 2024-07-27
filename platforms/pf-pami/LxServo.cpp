#include "LxServo.hpp"

#include <iostream>

namespace cogip {
namespace pf {
namespace actuators {
namespace servos {

std::ostream& operator << (std::ostream& os, Enum id) {
    os << static_cast<std::underlying_type_t<Enum>>(id);
    return os;
}

LxServo::LxServo(
    Enum id
) : Actuator(id) {
    lx_init(&lx_, lx_stream, static_cast<lx_id_t>(id));
}

lx_comm_error_t LxServo::move(
    const uint16_t position,
    const uint16_t time
) {
    std::cout << "INFO: LXMotor " << lx_.id << " at position " << position << std::endl;
    command_ = position;
    return lx_servo_move_time_write(&lx_, position, time);
}

lx_comm_error_t LxServo::move_wait(
    const uint16_t position,
    const uint16_t time
) {
    command_ = position;
    return lx_servo_move_time_wait_write(&lx_, position, time);
}

lx_comm_error_t LxServo::move_start() {
    return lx_servo_move_start(&lx_);
}

lx_comm_error_t LxServo::move_stop() {
    return lx_servo_move_stop(&lx_);
}

int16_t LxServo::pos_read() const {
    int16_t pos = 0;
    lx_comm_error_t ret = lx_servo_pos_read(&lx_, &pos);
    if (ret != LX_OK) {
        return -1;
    }
    return pos;
}

void LxServo::pb_copy(PB_Servo & pb_servo) const {
    pb_servo.set_id(static_cast<PB_ServoEnum>(id_));
    pb_servo.set_position(pos_read());
    pb_servo.set_command(command_);
}

void LxServo::send_state(void) {
    static int16_t _previous_position = pos_read();

    int16_t current_position = pos_read();

    if ((send_state_cb_) && (_previous_position != current_position)) {
        send_state_cb_(id_);
    }

    _previous_position = current_position;
}

uart_half_duplex_t * LxServo::lx_stream = nullptr;

} // namespace servos
} // namespace actuators
} // namespace pf
} // namespace cogip
