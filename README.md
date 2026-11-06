# motor_bridge

C++ control stack for the Seeed reBot Arm B601-DM (Damiao motors), built directly
on `motorbridge`'s `libmotor_abi.so` ABI — no Python, no gateway process at runtime.

## Build

```bash
./setup.sh              # Jetson / Ubuntu 22.04 arm64: installs all deps + motorbridge
mkdir -p build && cd build
cmake ..
make -j$(nproc)
```

On other platforms, install what `setup.sh` lists yourself (Qt6 or Qt5, SDL2, Eigen3,
`pip install motorbridge`) and skip straight to `cmake`/`make`.

## What's here

| Path | What |
|---|---|
| `include/motor_bridge.hpp`, `src/motor_bridge.cpp` | RAII C++ wrapper (`Controller`/`Motor`) over `libmotor_abi.so` |
| `gui/` | Qt app: per-joint sliders, gamepad control (Joint-Jog + Task-Space modes), live telemetry — `build/gui/motor_bridge_gui` |
| `gui/ArmKinematics.*` | Forward kinematics + Jacobian for task-space control, built from the arm's real URDF |
| `examples/test_main.cpp` | Minimal connectivity test — `build/motor_bridge_test` |
| `examples/test_kinematics.cpp` | Numerical validation of the kinematics — `build/test_kinematics` |

## Running

```bash
build/gui/motor_bridge_gui
```

Connect to the USB-CAN adapter (default `/dev/ttyACM0` @ 921600 baud), **Scan** to confirm
real motor IDs, **Enable** a joint before commanding it. Gamepad button mapping and mode
help text are shown in the app itself.

If `/dev/ttyACM0` isn't accessible, add yourself to `dialout` (`setup.sh` does this) and
re-login.

**Safety:** motors apply holding torque once Enabled. E-STOP (GUI button, or gamepad
Back — held, any mode) disables everything immediately.
