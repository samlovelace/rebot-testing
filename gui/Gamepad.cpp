#include "Gamepad.hpp"

#include <algorithm>
#include <cmath>

namespace {
constexpr float kAxisDeadzone = 0.12f;
}

Gamepad::Gamepad() {
    // SDL installs its own SIGINT/SIGTERM handlers by default, which would
    // swallow Ctrl-C / kill into an unhandled SDL_QUIT event instead of
    // letting the process exit normally. Disable that -- Qt owns the event loop.
    SDL_SetHint(SDL_HINT_NO_SIGNAL_HANDLERS, "1");
    SDL_SetHint(SDL_HINT_JOYSTICK_ALLOW_BACKGROUND_EVENTS, "1");
    SDL_InitSubSystem(SDL_INIT_GAMECONTROLLER | SDL_INIT_EVENTS);
    tryOpenFirst();
}

Gamepad::~Gamepad() {
    if (controller_) {
        SDL_GameControllerClose(controller_);
        controller_ = nullptr;
    }
    SDL_QuitSubSystem(SDL_INIT_GAMECONTROLLER | SDL_INIT_EVENTS);
}

void Gamepad::tryOpenFirst() {
    for (int i = 0; i < SDL_NumJoysticks(); ++i) {
        if (SDL_IsGameController(i)) {
            controller_ = SDL_GameControllerOpen(i);
            if (controller_) return;
        }
    }
}

void Gamepad::poll() {
    SDL_Event event;
    while (SDL_PollEvent(&event)) {
        if (event.type == SDL_CONTROLLERDEVICEADDED) {
            if (!controller_) controller_ = SDL_GameControllerOpen(event.cdevice.which);
        } else if (event.type == SDL_CONTROLLERDEVICEREMOVED) {
            if (controller_) {
                SDL_Joystick* js = SDL_GameControllerGetJoystick(controller_);
                if (js && SDL_JoystickInstanceID(js) == event.cdevice.which) {
                    SDL_GameControllerClose(controller_);
                    controller_ = nullptr;
                }
            }
        }
    }
    if (!controller_) tryOpenFirst();
}

bool Gamepad::isConnected() const { return controller_ != nullptr; }

QString Gamepad::name() const {
    if (!controller_) return "not connected";
    const char* n = SDL_GameControllerName(controller_);
    return n ? QString::fromUtf8(n) : "unknown controller";
}

bool Gamepad::buttonPressed(SDL_GameControllerButton button) const {
    if (!controller_) return false;
    return SDL_GameControllerGetButton(controller_, button) != 0;
}

float Gamepad::axisNormalized(SDL_GameControllerAxis axis) const {
    if (!controller_) return 0.0f;
    Sint16 raw = SDL_GameControllerGetAxis(controller_, axis);
    float v = raw / 32767.0f;
    if (std::fabs(v) < kAxisDeadzone) return 0.0f;
    return std::clamp(v, -1.0f, 1.0f);
}

QString Gamepad::buttonName(SDL_GameControllerButton button) {
    switch (button) {
        case SDL_CONTROLLER_BUTTON_A: return "A";
        case SDL_CONTROLLER_BUTTON_B: return "B";
        case SDL_CONTROLLER_BUTTON_X: return "X";
        case SDL_CONTROLLER_BUTTON_Y: return "Y";
        case SDL_CONTROLLER_BUTTON_LEFTSHOULDER: return "LB";
        case SDL_CONTROLLER_BUTTON_RIGHTSHOULDER: return "RB";
        case SDL_CONTROLLER_BUTTON_BACK: return "Back";
        case SDL_CONTROLLER_BUTTON_START: return "Start";
        case SDL_CONTROLLER_BUTTON_LEFTSTICK: return "L-Click";
        case SDL_CONTROLLER_BUTTON_RIGHTSTICK: return "R-Click";
        case SDL_CONTROLLER_BUTTON_DPAD_UP: return "D-Up";
        case SDL_CONTROLLER_BUTTON_DPAD_DOWN: return "D-Down";
        case SDL_CONTROLLER_BUTTON_DPAD_LEFT: return "D-Left";
        case SDL_CONTROLLER_BUTTON_DPAD_RIGHT: return "D-Right";
        default: return "?";
    }
}

float JogSource::value(const Gamepad& gp) const {
    switch (kind) {
        case Kind::ButtonPair: {
            float v = 0.0f;
            if (gp.buttonPressed(posButton)) v += 1.0f;
            if (gp.buttonPressed(negButton)) v -= 1.0f;
            return v;
        }
        case Kind::AxisPair: {
            // Trigger axes read [0, 1]; combine two unidirectional axes
            // into one bidirectional value.
            return gp.axisNormalized(posAxis) - gp.axisNormalized(negAxis);
        }
        case Kind::SingleAxis: {
            float v = gp.axisNormalized(axis);
            return invert ? -v : v;
        }
    }
    return 0.0f;
}
