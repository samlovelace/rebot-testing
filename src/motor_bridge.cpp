#include "motor_bridge.hpp"
#include "motor_abi_c.h"

#include <utility>

namespace motorbridge {

namespace {

[[noreturn]] void throw_last_error(const std::string& what) {
    const char* msg = motor_last_error_message();
    throw MotorBridgeError(what + " failed: " + (msg ? msg : "unknown error"));
}

void check(int32_t rc, const std::string& what) {
    if (rc != 0) throw_last_error(what);
}

} // namespace

// -- Motor -------------------------------------------------------------

Motor::~Motor() { close(); }

Motor::Motor(Motor&& other) noexcept : handle_(other.handle_) {
    other.handle_ = nullptr;
}

Motor& Motor::operator=(Motor&& other) noexcept {
    if (this != &other) {
        close();
        handle_ = other.handle_;
        other.handle_ = nullptr;
    }
    return *this;
}

void Motor::close() {
    if (handle_) {
        motor_handle_free(handle_);
        handle_ = nullptr;
    }
}

void Motor::enable() { check(motor_handle_enable(handle_), "enable"); }
void Motor::disable() { check(motor_handle_disable(handle_), "disable"); }
void Motor::clear_error() { check(motor_handle_clear_error(handle_), "clear_error"); }
void Motor::set_zero_position() { check(motor_handle_set_zero_position(handle_), "set_zero_position"); }

void Motor::ensure_mode(Mode mode, uint32_t timeout_ms) {
    check(motor_handle_ensure_mode(handle_, static_cast<uint32_t>(mode), timeout_ms), "ensure_mode");
}

void Motor::send_mit(float pos, float vel, float kp, float kd, float tau) {
    check(motor_handle_send_mit(handle_, pos, vel, kp, kd, tau), "send_mit");
}

void Motor::send_pos_vel(float pos, float vlim) {
    check(motor_handle_send_pos_vel(handle_, pos, vlim), "send_pos_vel");
}

void Motor::send_vel(float vel) {
    check(motor_handle_send_vel(handle_, vel), "send_vel");
}

void Motor::send_force_pos(float pos, float vlim, float ratio) {
    check(motor_handle_send_force_pos(handle_, pos, vlim, ratio), "send_force_pos");
}

void Motor::request_feedback() { check(motor_handle_request_feedback(handle_), "request_feedback"); }

void Motor::set_can_timeout_ms(uint32_t timeout_ms) {
    check(motor_handle_set_can_timeout_ms(handle_, timeout_ms), "set_can_timeout_ms");
}

void Motor::store_parameters() { check(motor_handle_store_parameters(handle_), "store_parameters"); }

void Motor::write_register_f32(uint8_t rid, float value) {
    check(motor_handle_write_register_f32(handle_, rid, value), "write_register_f32");
}

void Motor::write_register_u32(uint8_t rid, uint32_t value) {
    check(motor_handle_write_register_u32(handle_, rid, value), "write_register_u32");
}

float Motor::get_register_f32(uint8_t rid, uint32_t timeout_ms) {
    float out = 0.0f;
    check(motor_handle_get_register_f32(handle_, rid, timeout_ms, &out), "get_register_f32");
    return out;
}

uint32_t Motor::get_register_u32(uint8_t rid, uint32_t timeout_ms) {
    uint32_t out = 0;
    check(motor_handle_get_register_u32(handle_, rid, timeout_ms, &out), "get_register_u32");
    return out;
}

float Motor::damiao_get_param_f32(uint16_t param_id, uint32_t timeout_ms) {
    float out = 0.0f;
    check(motor_handle_damiao_get_param_f32(handle_, param_id, timeout_ms, &out), "damiao_get_param_f32");
    return out;
}

uint32_t Motor::damiao_get_param_u32(uint16_t param_id, uint32_t timeout_ms) {
    uint32_t out = 0;
    check(motor_handle_damiao_get_param_u32(handle_, param_id, timeout_ms, &out), "damiao_get_param_u32");
    return out;
}

void Motor::damiao_write_param_f32(uint16_t param_id, float value) {
    check(motor_handle_damiao_write_param_f32(handle_, param_id, value), "damiao_write_param_f32");
}

void Motor::damiao_write_param_u32(uint16_t param_id, uint32_t value) {
    check(motor_handle_damiao_write_param_u32(handle_, param_id, value), "damiao_write_param_u32");
}

std::optional<MotorState> Motor::get_state() {
    CState st{};
    check(motor_handle_get_state(handle_, &st), "get_state");
    if (!st.has_value) return std::nullopt;
    MotorState out;
    out.can_id = st.can_id;
    out.arbitration_id = st.arbitration_id;
    out.status_code = st.status_code;
    out.pos = st.pos;
    out.vel = st.vel;
    out.torq = st.torq;
    out.t_mos = st.t_mos;
    out.t_rotor = st.t_rotor;
    return out;
}

// -- Controller ----------------------------------------------------------

Controller Controller::from_dm_serial(const std::string& serial_port, uint32_t baud) {
    void* ptr = motor_controller_new_dm_serial(serial_port.c_str(), baud);
    if (!ptr) throw_last_error("new_dm_serial");
    return Controller(ptr);
}

Controller Controller::from_socketcan(const std::string& channel) {
    void* ptr = motor_controller_new_socketcan(channel.c_str());
    if (!ptr) throw_last_error("new_socketcan");
    return Controller(ptr);
}

Controller Controller::from_socketcanfd(const std::string& channel) {
    void* ptr = motor_controller_new_socketcanfd(channel.c_str());
    if (!ptr) throw_last_error("new_socketcanfd");
    return Controller(ptr);
}

Controller::~Controller() { close(); }

Controller::Controller(Controller&& other) noexcept : ptr_(other.ptr_) {
    other.ptr_ = nullptr;
}

Controller& Controller::operator=(Controller&& other) noexcept {
    if (this != &other) {
        close();
        ptr_ = other.ptr_;
        other.ptr_ = nullptr;
    }
    return *this;
}

void Controller::close() {
    if (ptr_) {
        motor_controller_free(ptr_);
        ptr_ = nullptr;
    }
}

void Controller::shutdown() { check(motor_controller_shutdown(ptr_), "controller_shutdown"); }
void Controller::close_bus() { check(motor_controller_close_bus(ptr_), "controller_close_bus"); }
void Controller::enable_all() { check(motor_controller_enable_all(ptr_), "enable_all"); }
void Controller::disable_all() { check(motor_controller_disable_all(ptr_), "disable_all"); }
void Controller::poll_feedback_once() { check(motor_controller_poll_feedback_once(ptr_), "poll_feedback_once"); }

Motor Controller::add_damiao_motor(uint16_t motor_id, uint16_t feedback_id, const std::string& model) {
    void* m = motor_controller_add_damiao_motor(ptr_, motor_id, feedback_id, model.c_str());
    if (!m) throw_last_error("add_damiao_motor");
    return Motor(m);
}

} // namespace motorbridge
