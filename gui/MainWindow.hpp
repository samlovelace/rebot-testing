#pragma once

#include <QMainWindow>
#include <QTimer>
#include <SDL2/SDL.h>
#include <array>
#include <optional>
#include <vector>

#include "ArmKinematics.hpp"
#include "Gamepad.hpp"
#include "motor_bridge.hpp"

class QLineEdit;
class QPushButton;
class QSlider;
class QLabel;
class QPlainTextEdit;

struct JointConfig {
    QString name;
    uint16_t motor_id;
    uint16_t feedback_id;
    JogSource jog;         // bidirectional gamepad input for Joint-Jog mode
    QString padLabel;      // display string, e.g. "LT-/RT+"
    double minRad;         // real joint limits from ReBot_Arm_DM.urdf
    double maxRad;
};

struct JointPanel {
    JointConfig cfg;
    std::optional<motorbridge::Motor> motor;
    bool enabled = false;

    QPushButton* enableBtn = nullptr;
    QSlider* slider = nullptr;       // integer millirad
    QLineEdit* kpEdit = nullptr;
    QLineEdit* kdEdit = nullptr;
    QLineEdit* jogRateEdit = nullptr; // rad/s while button held (Joint-Jog mode)
    QLabel* targetLabel = nullptr;   // commanded position (= slider value)
    QLabel* actualLabel = nullptr;   // measured pos/vel/torque/temps from get_state()
    QLabel* errorLabel = nullptr;    // target - actual, highlighted if large
    QLabel* statusLabel = nullptr;   // connection/error text
};

enum class ControlMode { JointJog, TaskSpace };
enum class EEFrame { Tool, Base };
enum class SpeedScale { Low, Medium };

class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    explicit MainWindow(QWidget* parent = nullptr);

private slots:
    void onConnect();
    void onDisconnect();
    void onEStop();
    void onScan();
    void onTick();
    void onModeToggled();

private:
    void enableJoint(JointPanel& jp);
    void disableJoint(JointPanel& jp);
    void toggleEnable(JointPanel& jp);
    void disableAll();
    void enableAllArmJoints();
    void log(const QString& text);
    void applyJog(JointPanel& jp, const JogSource& jog);
    void applyTaskSpaceJog();
    void updateModeStatusLabel();
    void startGoToPose(const std::array<double, 7>& pose);
    void applyGoToPose();

    std::optional<motorbridge::Controller> ctrl_;
    std::vector<JointPanel> joints_;      // 6 arm joints + 1 gripper, in that order
    Gamepad gamepad_;
    motorbridge::ArmKinematics kin_;

    ControlMode controlMode_ = ControlMode::JointJog;
    EEFrame eeFrame_ = EEFrame::Tool;
    SpeedScale speedScale_ = SpeedScale::Low;
    double speedTrim_ = 1.0;

    // rising-edge detection for toggle-style pad buttons
    bool prevX_ = false;
    bool prevY_ = false;
    bool prevDpadUp_ = false;
    bool prevDpadDown_ = false;
    bool prevDpadLeft_ = false;
    bool prevStart_ = false;

    std::optional<std::array<double, 7>> goalPose_;   // active go-to-pose target, if any

    QLineEdit* portEdit_ = nullptr;
    QLineEdit* baudEdit_ = nullptr;
    QLineEdit* modelEdit_ = nullptr;
    QLineEdit* scanStartEdit_ = nullptr;
    QLineEdit* scanEndEdit_ = nullptr;
    QPushButton* connectBtn_ = nullptr;
    QPushButton* disconnectBtn_ = nullptr;
    QPushButton* modeToggleBtn_ = nullptr;
    QPushButton* gotoPoseBtn_ = nullptr;
    QLabel* controllerStatusLabel_ = nullptr;
    QLabel* modeStatusLabel_ = nullptr;
    QPlainTextEdit* logView_ = nullptr;

    QTimer tickTimer_;
};
