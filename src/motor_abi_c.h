#pragma once
// Private extern "C" declarations for libmotor_abi.so.
// Signatures mirror motorbridge/abi.py's ctypes bindings exactly
// (verified against `nm -D libmotor_abi.so` on the installed package).
// Not a vendor-shipped header -- keep in sync if motorbridge is upgraded.

#include <cstdint>

extern "C" {

const char* motor_last_error_message();
const char* motor_abi_version();
const char* motor_abi_capabilities_json();

struct CState {
    int32_t has_value;
    uint8_t can_id;
    uint32_t arbitration_id;
    uint8_t status_code;
    float pos;
    float vel;
    float torq;
    float t_mos;
    float t_rotor;
};

void* motor_controller_new_socketcan(const char* channel);
void* motor_controller_new_socketcanfd(const char* channel);
void* motor_controller_new_dm_serial(const char* serial_port, uint32_t baud);
void* motor_controller_new_dm_device(const char* dm_device_type, const char* dm_channel);
void* motor_controller_new_mcu_serial(const char* serial_port, uint32_t baud);
void  motor_controller_free(void* ctrl);
int32_t motor_controller_poll_feedback_once(void* ctrl);
int32_t motor_controller_enable_all(void* ctrl);
int32_t motor_controller_disable_all(void* ctrl);
int32_t motor_controller_shutdown(void* ctrl);
int32_t motor_controller_close_bus(void* ctrl);

void* motor_controller_add_damiao_motor(void* ctrl, uint16_t motor_id, uint16_t feedback_id, const char* model);

void  motor_handle_free(void* motor);
int32_t motor_handle_enable(void* motor);
int32_t motor_handle_disable(void* motor);
int32_t motor_handle_clear_error(void* motor);
int32_t motor_handle_set_zero_position(void* motor);
int32_t motor_handle_ensure_mode(void* motor, uint32_t mode, uint32_t timeout_ms);

int32_t motor_handle_send_mit(void* motor, float pos, float vel, float kp, float kd, float tau);
int32_t motor_handle_send_pos_vel(void* motor, float pos, float vlim);
int32_t motor_handle_send_vel(void* motor, float vel);
int32_t motor_handle_send_force_pos(void* motor, float pos, float vlim, float ratio);

int32_t motor_handle_store_parameters(void* motor);
int32_t motor_handle_request_feedback(void* motor);
int32_t motor_handle_set_can_timeout_ms(void* motor, uint32_t timeout_ms);

int32_t motor_handle_write_register_f32(void* motor, uint8_t rid, float value);
int32_t motor_handle_write_register_u32(void* motor, uint8_t rid, uint32_t value);
int32_t motor_handle_get_register_f32(void* motor, uint8_t rid, uint32_t timeout_ms, float* out);
int32_t motor_handle_get_register_u32(void* motor, uint8_t rid, uint32_t timeout_ms, uint32_t* out);

int32_t motor_handle_damiao_get_param_f32(void* motor, uint16_t param_id, uint32_t timeout_ms, float* out);
int32_t motor_handle_damiao_get_param_u32(void* motor, uint16_t param_id, uint32_t timeout_ms, uint32_t* out);
int32_t motor_handle_damiao_write_param_f32(void* motor, uint16_t param_id, float value);
int32_t motor_handle_damiao_write_param_u32(void* motor, uint16_t param_id, uint32_t value);

int32_t motor_handle_get_state(void* motor, CState* out);

} // extern "C"
