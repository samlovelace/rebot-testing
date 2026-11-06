#!/usr/bin/env bash
# Install build/runtime dependencies for motor_bridge on Ubuntu 22.04
# (x86_64 dev machine, or a Jetson / arm64 target). Safe to re-run.
#
# Covers: C++ toolchain + CMake, Qt (6, falling back to 5), SDL2 (gamepad
# support), Eigen3 (task-space kinematics), the motorbridge Python package
# (which bundles the libmotor_abi.so this project links against), and serial
# port permissions for the USB-CAN adapter.
#
# apt package names are architecture-agnostic (apt resolves to the right
# binary for whatever machine this runs on) and `pip install motorbridge`
# already selects the matching wheel (manylinux_x86_64 vs manylinux_aarch64)
# on its own. The one thing that genuinely depends on architecture is the
# sanity check after install, below, which has to know what to expect.

set -euo pipefail

log() { echo -e "\033[1;36m==>\033[0m $*"; }
warn() { echo -e "\033[1;33m!!\033[0m $*" >&2; }
err() { echo -e "\033[1;31mERROR:\033[0m $*" >&2; }

ARCH="$(uname -m)"
case "$ARCH" in
    x86_64)  EXPECTED_FILE_ARCH="x86-64" ;;
    aarch64) EXPECTED_FILE_ARCH="ARM aarch64" ;;
    *)
        err "Unsupported architecture '$ARCH'. motorbridge only ships prebuilt wheels" \
            " for x86_64 and aarch64 Linux; there's nothing for this machine to install."
        exit 1
        ;;
esac
log "Detected architecture: $ARCH"

if ! grep -qi "jammy\|22.04" /etc/os-release 2>/dev/null; then
    warn "This doesn't look like Ubuntu 22.04 (jammy). Continuing anyway, but package" \
         " names below (esp. qt6-base-dev) assume jammy's repos."
fi

log "Updating apt and installing build toolchain + library dependencies"
sudo apt-get update
sudo apt-get install -y \
    build-essential \
    cmake \
    pkg-config \
    python3 \
    python3-pip \
    libsdl2-dev \
    libeigen3-dev

log "Installing Qt (preferring Qt6, falling back to Qt5 if unavailable)"
if sudo apt-get install -y qt6-base-dev qt6-base-dev-tools; then
    log "Qt6 installed."
elif sudo apt-get install -y qtbase5-dev qtbase5-dev-tools; then
    warn "Qt6 wasn't available; installed Qt5 instead. The GUI's CMakeLists" \
         " already falls back to Qt5 automatically."
else
    err "Could not install either Qt6 or Qt5 base-dev packages. Aborting."
    exit 1
fi

log "Installing the motorbridge Python package (ships libmotor_abi.so this project links against)"
python3 -m pip install --user --upgrade motorbridge

MOTOR_ABI_LIB="$(python3 -c 'import motorbridge, os; print(os.path.join(os.path.dirname(motorbridge.__file__), "lib", "libmotor_abi.so"))' 2>/dev/null || true)"
if [[ -z "$MOTOR_ABI_LIB" || ! -f "$MOTOR_ABI_LIB" ]]; then
    err "motorbridge installed but libmotor_abi.so was not found where expected."
    err "This usually means pip fell back to building from source (no prebuilt wheel"
    err "for this python/arch combo) and that build failed, or failed silently."
    err "Check: python3 -m pip install --user --upgrade motorbridge  (read its output)"
    exit 1
fi
log "Found libmotor_abi.so at: $MOTOR_ABI_LIB"

LIB_ARCH="$(file -b "$MOTOR_ABI_LIB" | grep -oE 'ARM aarch64|x86-64' || echo unknown)"
if [[ "$LIB_ARCH" != "$EXPECTED_FILE_ARCH" ]]; then
    warn "libmotor_abi.so reports as '$LIB_ARCH', expected '$EXPECTED_FILE_ARCH' for this" \
         " $ARCH machine. It was built for the wrong architecture (or 'file' couldn't" \
         " tell) -- compiling against it will likely fail to link. Re-check the pip" \
         " install above."
else
    log "libmotor_abi.so matches this machine's architecture ($ARCH) -- good."
fi

log "Adding $USER to the 'dialout' group (serial access to the USB-CAN adapter)"
if groups "$USER" | grep -qw dialout; then
    log "$USER is already in dialout."
else
    sudo usermod -aG dialout "$USER"
    warn "Added $USER to dialout. This only takes effect on your NEXT login/shell" \
         " (log out and back in, or run 'newgrp dialout' in this shell)."
fi

cat <<'EOF'

==> Setup complete. To build:

    mkdir -p build && cd build
    cmake ..
    make -j"$(nproc)"

Binaries land in build/ (motor_bridge_test, test_kinematics, gui/motor_bridge_gui).
If cmake can't find motorbridge's .so, re-check `python3 -m pip show motorbridge`
or pass it explicitly: cmake .. -DMOTOR_ABI_LIB=/path/to/libmotor_abi.so
EOF
