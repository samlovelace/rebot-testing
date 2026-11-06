#pragma once
// RAII C++ wrapper around libmotor_abi.so (the same flat C ABI the
// motorbridge Python bindings call). No Python, no gateway process.

#include <cstdint>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>

namespace motorbridge {

enum class Mode : uint32_t {
    MIT = 1,
    POS_VEL = 2,
    VEL = 3,
    FORCE_POS = 4,
    ROBSTRIDE_POS_VEL_CSP = 5,
};

struct MotorState {
    uint8_t can_id = 0;
    uint32_t arbitration_id = 0;
    uint8_t status_code = 0;
    float pos = 0.0f;
    float vel = 0.0f;
    float torq = 0.0f;
    float t_mos = 0.0f;
    float t_rotor = 0.0f;
};

class MotorBridgeError : public std::runtime_error {
public:
    explicit MotorBridgeError(const std::string& what) : std::runtime_error(what) {}
};

class Controller;

// Wraps one motor_handle_t*. Freed by its owning Controller's lifetime rules:
// the handle itself is independently reference-counted by the ABI, so a
// Motor can outlive the call that created it, but not the Controller's bus.
class Motor {
public:
    Motor() = default;
    ~Motor();

    Motor(Motor&& other) noexcept;
    Motor& operator=(Motor&& other) noexcept;
    Motor(const Motor&) = delete;
    Motor& operator=(const Motor&) = delete;

    void enable();
    void disable();
    void clear_error();
    void set_zero_position();
    void ensure_mode(Mode mode, uint32_t timeout_ms = 1000);

    void send_mit(float pos, float vel, float kp, float kd, float tau);
    void send_pos_vel(float pos, float vlim);
    void send_vel(float vel);
    void send_force_pos(float pos, float vlim, float ratio);

    void request_feedback();
    void set_can_timeout_ms(uint32_t timeout_ms);
    void store_parameters();

    void write_register_f32(uint8_t rid, float value);
    void write_register_u32(uint8_t rid, uint32_t value);
    float get_register_f32(uint8_t rid, uint32_t timeout_ms = 1000);
    uint32_t get_register_u32(uint8_t rid, uint32_t timeout_ms = 1000);

    // Damiao-specific parameter registers (model limits, PI gains, etc.)
    float damiao_get_param_f32(uint16_t param_id, uint32_t timeout_ms = 1000);
    uint32_t damiao_get_param_u32(uint16_t param_id, uint32_t timeout_ms = 1000);
    void damiao_write_param_f32(uint16_t param_id, float value);
    void damiao_write_param_u32(uint16_t param_id, uint32_t value);

    // Latest feedback from the controller's receive buffer, if any has
    // arrived since the last poll_feedback_once() on the owning Controller.
    std::optional<MotorState> get_state();

    bool valid() const { return handle_ != nullptr; }

private:
    friend class Controller;
    explicit Motor(void* handle) : handle_(handle) {}
    void close();
    void* handle_ = nullptr;
};

class Controller {
public:
    static Controller from_dm_serial(const std::string& serial_port, uint32_t baud = 921600);
    static Controller from_socketcan(const std::string& channel);
    static Controller from_socketcanfd(const std::string& channel);

    ~Controller();
    Controller(Controller&& other) noexcept;
    Controller& operator=(Controller&& other) noexcept;
    Controller(const Controller&) = delete;
    Controller& operator=(const Controller&) = delete;

    void shutdown();
    void close_bus();
    void enable_all();
    void disable_all();

    // Drains whatever has arrived on the transport since the last call and
    // updates each Motor's internal feedback cache. Call this once per
    // control-loop tick before reading get_state() on any motor.
    void poll_feedback_once();

    Motor add_damiao_motor(uint16_t motor_id, uint16_t feedback_id, const std::string& model);

    bool valid() const { return ptr_ != nullptr; }

private:
    explicit Controller(void* ptr) : ptr_(ptr) {}
    void close();
    void* ptr_ = nullptr;
};

} // namespace motorbridge
