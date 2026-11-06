// Minimal connectivity test: connects, enables motor id 1, holds position 0
// in MIT mode for a few seconds while printing feedback, then disables.
//
// Usage: ./motor_bridge_test [serial_port] [baud] [motor_id] [feedback_id] [model]

#include "motor_bridge.hpp"

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <thread>

using namespace motorbridge;
using namespace std::chrono_literals;

int main(int argc, char** argv) {
    std::string port = argc > 1 ? argv[1] : "/dev/ttyACM0";
    uint32_t baud = argc > 2 ? std::strtoul(argv[2], nullptr, 0) : 921600;
    uint16_t motor_id = argc > 3 ? static_cast<uint16_t>(std::strtoul(argv[3], nullptr, 0)) : 1;
    uint16_t feedback_id = argc > 4 ? static_cast<uint16_t>(std::strtoul(argv[4], nullptr, 0)) : (0x10 + motor_id);
    std::string model = argc > 5 ? argv[5] : "4340";

    try {
        Controller ctrl = Controller::from_dm_serial(port, baud);
        Motor motor = ctrl.add_damiao_motor(motor_id, feedback_id, model);

        motor.enable();
        motor.ensure_mode(Mode::MIT);
        std::printf("enabled motor id=0x%02X feedback_id=0x%02X model=%s on %s @ %u\n",
                    motor_id, feedback_id, model.c_str(), port.c_str(), baud);

        for (int i = 0; i < 100; ++i) {
            ctrl.poll_feedback_once();
            motor.send_mit(/*pos=*/0.0f, /*vel=*/0.0f, /*kp=*/10.0f, /*kd=*/1.0f, /*tau=*/0.0f);

            if (auto st = motor.get_state()) {
                std::printf("pos=%+.3f vel=%+.3f torq=%+.3f t_mos=%.0fC t_rotor=%.0fC\n",
                            st->pos, st->vel, st->torq, st->t_mos, st->t_rotor);
            } else {
                std::printf("(no feedback yet)\n");
            }
            std::this_thread::sleep_for(20ms);
        }

        motor.disable();
        ctrl.close_bus();
        std::printf("done\n");
    } catch (const MotorBridgeError& e) {
        std::fprintf(stderr, "motorbridge error: %s\n", e.what());
        return 1;
    }

    return 0;
}
