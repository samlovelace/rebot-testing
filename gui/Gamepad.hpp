#pragma once

#include <SDL2/SDL.h>
#include <QString>

// Thin wrapper around SDL's GameController API. SDL normalizes Xbox/
// PlayStation/generic Bluetooth pads into one virtual layout (A/B/X/Y,
// shoulders, triggers, stick clicks, dpad), so button assignments below
// work the same regardless of which controller is actually paired.
class Gamepad {
public:
    Gamepad();
    ~Gamepad();

    Gamepad(const Gamepad&) = delete;
    Gamepad& operator=(const Gamepad&) = delete;

    // Pumps the SDL event queue (handles connect/disconnect) and refreshes
    // button state. Call once per tick from the Qt timer.
    void poll();

    bool isConnected() const;
    QString name() const;
    bool buttonPressed(SDL_GameControllerButton button) const;

    // Normalized axis value in [-1, 1] (triggers naturally read [0, 1] since
    // the hardware never reports negative for them), with a small deadzone
    // applied so resting sticks don't cause drift.
    float axisNormalized(SDL_GameControllerAxis axis) const;

    static QString buttonName(SDL_GameControllerButton button);

private:
    void tryOpenFirst();

    SDL_GameController* controller_ = nullptr;
};

// One joint's bidirectional jog input: either a button pair, a pair of
// unidirectional trigger axes combined into one bidirectional value, or a
// single bidirectional stick axis. value() returns a signed [-1, 1] command
// that MainWindow scales by the joint's jog rate (rad/s) each tick.
struct JogSource {
    enum class Kind { ButtonPair, AxisPair, SingleAxis };

    Kind kind = Kind::ButtonPair;
    SDL_GameControllerButton posButton = SDL_CONTROLLER_BUTTON_INVALID;
    SDL_GameControllerButton negButton = SDL_CONTROLLER_BUTTON_INVALID;
    SDL_GameControllerAxis posAxis = SDL_CONTROLLER_AXIS_INVALID;  // AxisPair: e.g. right trigger
    SDL_GameControllerAxis negAxis = SDL_CONTROLLER_AXIS_INVALID;  // AxisPair: e.g. left trigger
    SDL_GameControllerAxis axis = SDL_CONTROLLER_AXIS_INVALID;     // SingleAxis
    bool invert = false;                                          // SingleAxis only

    static JogSource buttons(SDL_GameControllerButton pos, SDL_GameControllerButton neg) {
        JogSource j;
        j.kind = Kind::ButtonPair;
        j.posButton = pos;
        j.negButton = neg;
        return j;
    }

    static JogSource triggers(SDL_GameControllerAxis posAxis, SDL_GameControllerAxis negAxis) {
        JogSource j;
        j.kind = Kind::AxisPair;
        j.posAxis = posAxis;
        j.negAxis = negAxis;
        return j;
    }

    static JogSource stick(SDL_GameControllerAxis axis, bool invert = false) {
        JogSource j;
        j.kind = Kind::SingleAxis;
        j.axis = axis;
        j.invert = invert;
        return j;
    }

    float value(const Gamepad& gp) const;
};
