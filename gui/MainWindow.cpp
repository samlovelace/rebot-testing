#include "MainWindow.hpp"

#include <algorithm>

#include <QGridLayout>
#include <QGroupBox>
#include <QLabel>
#include <QLineEdit>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QScrollArea>
#include <QSizePolicy>
#include <QSlider>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QWheelEvent>

using motorbridge::Mode;
using motorbridge::MotorBridgeError;
using motorbridge::JointVec;
using motorbridge::Twist;
using motorbridge::kNumArmJoints;

namespace {
constexpr int kDtMs = 20;
constexpr int kSliderScale = 1000;   // slider units per radian
constexpr double kGripperLimitRad = 3.14159;  // no authoritative gripper range; generic placeholder

// Per-joint velocity ceilings applied as a hard safety backstop on top of
// whatever the task-space IK resolves -- real motor ratings are 3.77 rad/s
// (DM4340P, joints 1-3) and 12.57 rad/s (DM4310, joints 4-6); these defaults
// sit well under that (~10-20%). Joint-jog mode's jog-rate field is separate
// and user-editable per joint; this table only bounds task-space mode.
constexpr double kJointVelLow[kNumArmJoints]    = {0.4, 0.4, 0.4, 1.0, 1.0, 1.0};
constexpr double kJointVelMedium[kNumArmJoints] = {0.8, 0.8, 0.8, 2.0, 2.0, 2.0};

// Cartesian speed scale for task-space mode (m/s translation, rad/s rotation).
constexpr double kCartLinearLow = 0.05;
constexpr double kCartLinearMedium = 0.10;
constexpr double kCartAngularLow = 0.2;
constexpr double kCartAngularMedium = 0.4;
constexpr double kDlsLambda = 0.05;   // damped least-squares singularity guard

// EDIT THIS before trusting it -- motor_id/feedback_id are placeholders using
// the Damiao default convention feedback_id = 0x10 + motor_id. Use the Scan
// button to confirm. Position limits ARE real -- taken from the Seeed reBot
// Arm B601-DM's ReBot_Arm_DM.urdf <joint><limit> elements.
//
// Joint-Jog mode: each joint gets a bidirectional gamepad input (two
// buttons, two triggers, or one stick axis). "+" drives the slider up, "-"
// drives it down; flip the two arguments to a JogSource:: factory below to
// reverse any one of them. (Task-Space mode uses its own fixed mapping --
// see applyTaskSpaceJog().)
const JointConfig kDefaultJoints[] = {
    {"Joint 1", 1, 0x11,
     JogSource::triggers(SDL_CONTROLLER_AXIS_TRIGGERRIGHT, SDL_CONTROLLER_AXIS_TRIGGERLEFT),
     "LT-/RT+", -2.8, 2.8},
    {"Joint 2", 2, 0x12,
     JogSource::buttons(SDL_CONTROLLER_BUTTON_RIGHTSHOULDER, SDL_CONTROLLER_BUTTON_LEFTSHOULDER),
     "LB-/RB+", -3.14, 0.0},
    {"Joint 3", 3, 0x13,
     JogSource::buttons(SDL_CONTROLLER_BUTTON_X, SDL_CONTROLLER_BUTTON_A),
     "A-/X+", -3.14, 0.0},
    {"Joint 4", 4, 0x14,
     JogSource::buttons(SDL_CONTROLLER_BUTTON_Y, SDL_CONTROLLER_BUTTON_B),
     "B-/Y+", -1.87, 1.57},
    {"Joint 5", 5, 0x15,
     JogSource::stick(SDL_CONTROLLER_AXIS_LEFTY, /*invert=*/true),
     "L-Stick Y (up+/down-)", -1.57, 1.57},
    {"Joint 6", 6, 0x16,
     JogSource::stick(SDL_CONTROLLER_AXIS_RIGHTX, /*invert=*/false),
     "R-Stick X (right+/left-)", -3.14, 3.14},
    {"Gripper", 7, 0x17,
     JogSource::stick(SDL_CONTROLLER_AXIS_RIGHTY, /*invert=*/true),
     "R-Stick Y (up+/down-)", -kGripperLimitRad, kGripperLimitRad},
};

// Task-Space mode's gripper input (A=close, B=open) -- right stick is
// repurposed for pitch/yaw in this mode, so the gripper moves off it.
const JogSource kGripperTaskSpaceJog =
    JogSource::buttons(SDL_CONTROLLER_BUTTON_B, SDL_CONTROLLER_BUTTON_A);

// Preset "go to pose" target: [J1..J6, gripper] in radians. Within each
// joint's real URDF limit (checked against kDefaultJoints' minRad/maxRad).
const std::array<double, 7> kPresetPose = {0.0, -1.0, -0.5, -0.7, 0.0, 0.0, -2.0};
constexpr double kGotoVelLow = 0.4;    // rad/s cap for the gripper slot while moving to a preset
constexpr double kGotoToleranceRad = 0.01;
constexpr double kPosErrorWarnRad = 0.05;   // target/actual gap beyond this is highlighted

// QSlider normally eats mouse-wheel events to change its own value, which
// fights scrolling a QScrollArea full of sliders. Ignore wheel on the
// sliders so scrolling always scrolls the joint list instead.
class NoWheelSlider : public QSlider {
public:
    using QSlider::QSlider;
protected:
    void wheelEvent(QWheelEvent* event) override { event->ignore(); }
};

} // namespace

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent) {
    setWindowTitle("Damiao Joint Slider Test (Qt)");

    auto* central = new QWidget(this);
    auto* mainLayout = new QVBoxLayout(central);

    auto* warn = new QLabel(
        "CAUTION: motors apply holding torque once Enabled. "
        "Keep hands/objects clear of pinch points. Use E-STOP if anything looks wrong.");
    warn->setWordWrap(true);
    warn->setStyleSheet("background-color:#b33; color:white; padding:6px;");
    mainLayout->addWidget(warn);

    auto* connRow = new QHBoxLayout();
    connRow->addWidget(new QLabel("Port:"));
    portEdit_ = new QLineEdit("/dev/ttyACM0");
    portEdit_->setFixedWidth(120);
    connRow->addWidget(portEdit_);

    connRow->addWidget(new QLabel("Baud:"));
    baudEdit_ = new QLineEdit("921600");
    baudEdit_->setFixedWidth(80);
    connRow->addWidget(baudEdit_);

    connRow->addWidget(new QLabel("Model:"));
    modelEdit_ = new QLineEdit("4340");
    modelEdit_->setFixedWidth(60);
    connRow->addWidget(modelEdit_);

    connectBtn_ = new QPushButton("Connect");
    connect(connectBtn_, &QPushButton::clicked, this, &MainWindow::onConnect);
    connRow->addWidget(connectBtn_);

    disconnectBtn_ = new QPushButton("Disconnect");
    disconnectBtn_->setEnabled(false);
    connect(disconnectBtn_, &QPushButton::clicked, this, &MainWindow::onDisconnect);
    connRow->addWidget(disconnectBtn_);

    auto* estopBtn = new QPushButton("E-STOP (disable all)");
    estopBtn->setStyleSheet("background-color:#b33; color:white; font-weight:bold;");
    connect(estopBtn, &QPushButton::clicked, this, &MainWindow::onEStop);
    connRow->addWidget(estopBtn);

    modeToggleBtn_ = new QPushButton("Mode: Joint-Jog (click for Task-Space)");
    connect(modeToggleBtn_, &QPushButton::clicked, this, &MainWindow::onModeToggled);
    connRow->addWidget(modeToggleBtn_);

    gotoPoseBtn_ = new QPushButton("Go to Preset Pose (D-pad Left)");
    connect(gotoPoseBtn_, &QPushButton::clicked, this, [this]() { startGoToPose(kPresetPose); });
    connRow->addWidget(gotoPoseBtn_);
    connRow->addStretch();

    controllerStatusLabel_ = new QLabel("Controller: not connected");
    controllerStatusLabel_->setObjectName("controllerStatus");
    connRow->addWidget(controllerStatusLabel_);

    modeStatusLabel_ = new QLabel();
    modeStatusLabel_->setObjectName("controllerStatus");
    modeStatusLabel_->setVisible(false);
    connRow->addWidget(modeStatusLabel_);
    mainLayout->addLayout(connRow);

    auto* taskHelp = new QLabel(
        "Task-Space: L-stick=X/Y  LT/RT=Z  R-stick=pitch/yaw  LB/RB=roll  "
        "A/B=gripper close/open  X=toggle frame  Y=cycle speed  D-pad Up/Down=trim speed  "
        "D-pad Left=go to preset pose  Back=E-STOP (hold, any mode)  Start=enable all");
    taskHelp->setObjectName("padLabel");
    taskHelp->setWordWrap(true);
    mainLayout->addWidget(taskHelp);

    auto* scanRow = new QHBoxLayout();
    scanRow->addWidget(new QLabel("Scan IDs:"));
    scanStartEdit_ = new QLineEdit("1");
    scanStartEdit_->setFixedWidth(50);
    scanRow->addWidget(scanStartEdit_);
    scanRow->addWidget(new QLabel("to"));
    scanEndEdit_ = new QLineEdit("16");
    scanEndEdit_->setFixedWidth(50);
    scanRow->addWidget(scanEndEdit_);
    auto* scanBtn = new QPushButton("Scan");
    connect(scanBtn, &QPushButton::clicked, this, &MainWindow::onScan);
    scanRow->addWidget(scanBtn);
    scanRow->addStretch();
    mainLayout->addLayout(scanRow);

    logView_ = new QPlainTextEdit();
    logView_->setReadOnly(true);
    logView_->setFixedHeight(120);
    mainLayout->addWidget(logView_);

    auto* scroll = new QScrollArea();
    scroll->setWidgetResizable(true);
    auto* jointsContainer = new QWidget();
    auto* jointsLayout = new QVBoxLayout(jointsContainer);

    for (const auto& cfg : kDefaultJoints) {
        JointPanel jp;
        jp.cfg = cfg;

        auto* box = new QGroupBox(QString("%1 (id=0x%2)").arg(cfg.name).arg(cfg.motor_id, 2, 16, QChar('0')));
        auto* boxLayout = new QVBoxLayout(box);

        // Row 1: controls
        auto* controlRow = new QHBoxLayout();

        jp.enableBtn = new QPushButton("Enable");
        jp.enableBtn->setEnabled(false);
        jp.enableBtn->setFixedWidth(80);
        controlRow->addWidget(jp.enableBtn);

        jp.slider = new NoWheelSlider(Qt::Horizontal);
        jp.slider->setRange(static_cast<int>(cfg.minRad * kSliderScale), static_cast<int>(cfg.maxRad * kSliderScale));
        jp.slider->setValue(std::clamp(0, jp.slider->minimum(), jp.slider->maximum()));
        jp.slider->setMinimumWidth(150);
        jp.slider->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
        controlRow->addWidget(jp.slider, /*stretch=*/1);

        controlRow->addWidget(new QLabel("kp:"));
        jp.kpEdit = new QLineEdit("10.0");
        jp.kpEdit->setFixedWidth(50);
        controlRow->addWidget(jp.kpEdit);

        controlRow->addWidget(new QLabel("kd:"));
        jp.kdEdit = new QLineEdit("1.0");
        jp.kdEdit->setFixedWidth(50);
        controlRow->addWidget(jp.kdEdit);

        controlRow->addWidget(new QLabel("jog:"));
        jp.jogRateEdit = new QLineEdit("1.0");
        jp.jogRateEdit->setFixedWidth(45);
        jp.jogRateEdit->setToolTip("rad/s while the assigned button is held (Joint-Jog mode)");
        controlRow->addWidget(jp.jogRateEdit);

        auto* padLabel = new QLabel(QString("Pad: %1").arg(cfg.padLabel));
        padLabel->setObjectName("padLabel");
        controlRow->addWidget(padLabel);

        boxLayout->addLayout(controlRow);

        // Row 2: telemetry -- target (commanded) vs actual (measured), and
        // an explicit error readout so a joint that isn't tracking its
        // command is immediately visible instead of buried in a long string.
        auto* telemetryRow = new QHBoxLayout();

        jp.targetLabel = new QLabel("Target: --");
        jp.targetLabel->setObjectName("jointStatus");
        jp.targetLabel->setMinimumWidth(130);
        telemetryRow->addWidget(jp.targetLabel);

        jp.actualLabel = new QLabel("Actual: --");
        jp.actualLabel->setObjectName("jointStatus");
        jp.actualLabel->setMinimumWidth(130);
        telemetryRow->addWidget(jp.actualLabel);

        jp.errorLabel = new QLabel("Err: --");
        jp.errorLabel->setObjectName("jointStatus");
        jp.errorLabel->setMinimumWidth(110);
        telemetryRow->addWidget(jp.errorLabel);

        jp.statusLabel = new QLabel("--");
        jp.statusLabel->setObjectName("jointStatus");
        telemetryRow->addWidget(jp.statusLabel, /*stretch=*/1);

        boxLayout->addLayout(telemetryRow);

        jointsLayout->addWidget(box);
        joints_.push_back(std::move(jp));
    }
    jointsLayout->addStretch();
    scroll->setWidget(jointsContainer);
    scroll->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    mainLayout->addWidget(scroll, /*stretch=*/1);

    // wire enable buttons now that joints_ has stable storage (vector won't reallocate further)
    for (size_t i = 0; i < joints_.size(); ++i) {
        connect(joints_[i].enableBtn, &QPushButton::clicked, this, [this, i]() {
            toggleEnable(joints_[i]);
        });
    }

    central->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    setCentralWidget(central);
    resize(1200, 850);

    connect(&tickTimer_, &QTimer::timeout, this, &MainWindow::onTick);
    tickTimer_.start(kDtMs);   // runs always so gamepad status/jog work pre-connect too
}

void MainWindow::log(const QString& text) {
    logView_->appendPlainText(text);
}

void MainWindow::onConnect() {
    if (ctrl_) return;
    QString port = portEdit_->text().trimmed();
    bool ok = false;
    uint32_t baud = baudEdit_->text().trimmed().toUInt(&ok);
    if (!ok) {
        log("invalid baud");
        return;
    }
    QString model = modelEdit_->text().trimmed();
    if (model.isEmpty()) model = "4340";

    try {
        ctrl_.emplace(motorbridge::Controller::from_dm_serial(port.toStdString(), baud));
    } catch (const MotorBridgeError& e) {
        log(QString("connect failed: %1").arg(e.what()));
        return;
    }

    for (auto& jp : joints_) {
        try {
            jp.motor.emplace(ctrl_->add_damiao_motor(jp.cfg.motor_id, jp.cfg.feedback_id, model.toStdString()));
            jp.enableBtn->setEnabled(true);
            jp.statusLabel->setText("connected, not enabled");
        } catch (const MotorBridgeError& e) {
            jp.motor.reset();
            jp.statusLabel->setText(QString("add_motor failed: %1").arg(e.what()));
        }
    }

    log(QString("connected to %1 @ %2").arg(port).arg(baud));
    connectBtn_->setEnabled(false);
    disconnectBtn_->setEnabled(true);
}

void MainWindow::disableAll() {
    for (auto& jp : joints_) {
        disableJoint(jp);
    }
}

void MainWindow::onDisconnect() {
    disableAll();
    if (ctrl_) {
        for (auto& jp : joints_) {
            jp.motor.reset();
            jp.enableBtn->setEnabled(false);
            jp.statusLabel->setText("--");
            jp.targetLabel->setText("Target: --");
            jp.actualLabel->setText("Actual: --");
            jp.errorLabel->setText("Err: --");
            jp.errorLabel->setStyleSheet("");
        }
        try {
            ctrl_->close_bus();
        } catch (const MotorBridgeError&) {
        }
        ctrl_.reset();
    }
    log("disconnected");
    connectBtn_->setEnabled(true);
    disconnectBtn_->setEnabled(false);
}

void MainWindow::enableJoint(JointPanel& jp) {
    if (!jp.motor || jp.enabled) return;
    try {
        jp.motor->enable();
        jp.motor->ensure_mode(Mode::MIT);
    } catch (const MotorBridgeError& e) {
        log(QString("%1: enable failed: %2").arg(jp.cfg.name).arg(e.what()));
        return;
    }
    try {
        if (ctrl_) ctrl_->poll_feedback_once();
        if (auto st = jp.motor->get_state()) {
            jp.slider->setValue(static_cast<int>(st->pos * kSliderScale));
        }
    } catch (const MotorBridgeError&) {
    }
    jp.enabled = true;
    jp.enableBtn->setText("Disable");
}

void MainWindow::disableJoint(JointPanel& jp) {
    if (!jp.motor || !jp.enabled) return;
    try {
        jp.motor->disable();
    } catch (const MotorBridgeError& e) {
        log(QString("%1: disable failed: %2").arg(jp.cfg.name).arg(e.what()));
    }
    jp.enabled = false;
    jp.enableBtn->setText("Enable");
}

void MainWindow::toggleEnable(JointPanel& jp) {
    if (jp.enabled) {
        disableJoint(jp);
    } else {
        enableJoint(jp);
    }
}

void MainWindow::enableAllArmJoints() {
    for (auto& jp : joints_) {
        enableJoint(jp);
    }
}

void MainWindow::onEStop() {
    disableAll();
    log("E-STOP: all joints disabled");
}

void MainWindow::onScan() {
    bool ok1 = false, ok2 = false;
    int startId = scanStartEdit_->text().trimmed().toInt(&ok1, 0);
    int endId = scanEndEdit_->text().trimmed().toInt(&ok2, 0);
    if (!ok1 || !ok2 || endId < startId) {
        log("invalid scan range");
        return;
    }

    QString model = modelEdit_->text().trimmed();
    if (model.isEmpty()) model = "4340";

    bool ownCtrl = !ctrl_.has_value();
    std::optional<motorbridge::Controller> tempCtrl;
    motorbridge::Controller* ctrl = nullptr;

    if (ownCtrl) {
        try {
            tempCtrl.emplace(motorbridge::Controller::from_dm_serial(
                portEdit_->text().trimmed().toStdString(),
                baudEdit_->text().trimmed().toUInt()));
        } catch (const MotorBridgeError& e) {
            log(QString("scan: could not open port: %1").arg(e.what()));
            return;
        }
        ctrl = &*tempCtrl;
    } else {
        ctrl = &*ctrl_;
    }

    log(QString("scanning ids 0x%1..0x%2 ...").arg(startId, 0, 16).arg(endId, 0, 16));
    int found = 0;
    for (int mid = startId; mid <= endId; ++mid) {
        uint16_t fid = static_cast<uint16_t>(0x10 + mid);
        try {
            auto m = ctrl->add_damiao_motor(static_cast<uint16_t>(mid), fid, model.toStdString());
            try {
                uint32_t escId = m.get_register_u32(8, 150);
                log(QString("  id=0x%1: FOUND (esc_id=0x%2)").arg(mid, 2, 16, QChar('0')).arg(escId, 0, 16));
                ++found;
            } catch (const MotorBridgeError&) {
                log(QString("  id=0x%1: no reply").arg(mid, 2, 16, QChar('0')));
            }
        } catch (const MotorBridgeError& e) {
            log(QString("  id=0x%1: add_motor failed: %2").arg(mid, 2, 16, QChar('0')).arg(e.what()));
        }
    }
    log(QString("scan done: %1 motor(s) found").arg(found));

    if (ownCtrl && tempCtrl) {
        try {
            tempCtrl->close_bus();
        } catch (const MotorBridgeError&) {
        }
    }
}

void MainWindow::applyJog(JointPanel& jp, const JogSource& jog) {
    float v = jog.value(gamepad_);   // signed [-1, 1]; proportional for sticks/triggers
    if (v == 0.0f) return;
    bool okRate = false;
    float rate = jp.jogRateEdit->text().toFloat(&okRate);
    if (!okRate) return;
    int delta = static_cast<int>(v * rate * kSliderScale * (kDtMs / 1000.0f));
    if (delta == 0) return;
    int newVal = jp.slider->value() + delta;
    newVal = std::max(jp.slider->minimum(), std::min(jp.slider->maximum(), newVal));
    jp.slider->setValue(newVal);
}

void MainWindow::updateModeStatusLabel() {
    QString frame = (eeFrame_ == EEFrame::Tool) ? "Tool" : "Base";
    QString speed = (speedScale_ == SpeedScale::Low) ? "Low" : "Medium";
    modeStatusLabel_->setText(QString("Frame: %1 | Speed: %2 (x%3)")
                                   .arg(frame).arg(speed).arg(speedTrim_, 0, 'f', 1));
}

void MainWindow::onModeToggled() {
    controlMode_ = (controlMode_ == ControlMode::JointJog) ? ControlMode::TaskSpace : ControlMode::JointJog;
    if (controlMode_ == ControlMode::TaskSpace) {
        modeToggleBtn_->setText("Mode: Task-Space (click for Joint-Jog)");
        modeStatusLabel_->setVisible(true);
        updateModeStatusLabel();
        log("switched to Task-Space mode");
    } else {
        modeToggleBtn_->setText("Mode: Joint-Jog (click for Task-Space)");
        modeStatusLabel_->setVisible(false);
        log("switched to Joint-Jog mode");
    }
}

void MainWindow::applyTaskSpaceJog() {
    // --- toggle-style buttons (rising edge only) ---
    bool x = gamepad_.buttonPressed(SDL_CONTROLLER_BUTTON_X);
    if (x && !prevX_) eeFrame_ = (eeFrame_ == EEFrame::Tool) ? EEFrame::Base : EEFrame::Tool;
    prevX_ = x;

    bool y = gamepad_.buttonPressed(SDL_CONTROLLER_BUTTON_Y);
    if (y && !prevY_) speedScale_ = (speedScale_ == SpeedScale::Low) ? SpeedScale::Medium : SpeedScale::Low;
    prevY_ = y;

    bool dup = gamepad_.buttonPressed(SDL_CONTROLLER_BUTTON_DPAD_UP);
    if (dup && !prevDpadUp_) speedTrim_ = std::min(1.5, speedTrim_ + 0.1);
    prevDpadUp_ = dup;

    bool ddown = gamepad_.buttonPressed(SDL_CONTROLLER_BUTTON_DPAD_DOWN);
    if (ddown && !prevDpadDown_) speedTrim_ = std::max(0.5, speedTrim_ - 0.1);
    prevDpadDown_ = ddown;

    bool start = gamepad_.buttonPressed(SDL_CONTROLLER_BUTTON_START);
    if (start && !prevStart_) enableAllArmJoints();
    prevStart_ = start;

    updateModeStatusLabel();

    if (joints_.size() < static_cast<size_t>(kNumArmJoints) + 1) return;

    // gripper is the last panel; A=close, B=open regardless of frame/speed
    applyJog(joints_.back(), kGripperTaskSpaceJog);

    const double linMax = (speedScale_ == SpeedScale::Low ? kCartLinearLow : kCartLinearMedium) * speedTrim_;
    const double angMax = (speedScale_ == SpeedScale::Low ? kCartAngularLow : kCartAngularMedium) * speedTrim_;

    const double vx = -gamepad_.axisNormalized(SDL_CONTROLLER_AXIS_LEFTY) * linMax;   // stick up = forward+
    const double vy = -gamepad_.axisNormalized(SDL_CONTROLLER_AXIS_LEFTX) * linMax;   // stick right = right+
    const double vz = (gamepad_.axisNormalized(SDL_CONTROLLER_AXIS_TRIGGERRIGHT) -
                        gamepad_.axisNormalized(SDL_CONTROLLER_AXIS_TRIGGERLEFT)) * linMax;

    double wx = 0.0;
    if (gamepad_.buttonPressed(SDL_CONTROLLER_BUTTON_RIGHTSHOULDER)) wx += 1.0;
    if (gamepad_.buttonPressed(SDL_CONTROLLER_BUTTON_LEFTSHOULDER)) wx -= 1.0;
    wx *= angMax;
    const double wy = -gamepad_.axisNormalized(SDL_CONTROLLER_AXIS_RIGHTY) * angMax;  // stick up = pitch+
    const double wz = gamepad_.axisNormalized(SDL_CONTROLLER_AXIS_RIGHTX) * angMax;   // stick right = yaw+

    Twist twist_local;
    twist_local << vx, vy, vz, wx, wy, wz;
    if (twist_local.isZero(1e-9)) return;

    JointVec q;
    for (int i = 0; i < kNumArmJoints; ++i) {
        q(i) = joints_[i].slider->value() / static_cast<double>(kSliderScale);
    }

    Twist twist_base;
    if (eeFrame_ == EEFrame::Tool) {
        const Eigen::Isometry3d eeT = kin_.forwardKinematics(q);
        twist_base.head<3>() = eeT.linear() * twist_local.head<3>();
        twist_base.tail<3>() = eeT.linear() * twist_local.tail<3>();
    } else {
        twist_base = twist_local;
    }

    const JointVec qdot = kin_.resolvedRateJointVelocity(q, twist_base, kDlsLambda);
    const double* velLimit = (speedScale_ == SpeedScale::Low) ? kJointVelLow : kJointVelMedium;
    const double dt = kDtMs / 1000.0;

    for (int i = 0; i < kNumArmJoints; ++i) {
        const double qd = std::clamp(qdot(i), -velLimit[i], velLimit[i]);
        const int delta = static_cast<int>(qd * dt * kSliderScale);
        if (delta == 0) continue;
        int newVal = joints_[i].slider->value() + delta;
        newVal = std::max(joints_[i].slider->minimum(), std::min(joints_[i].slider->maximum(), newVal));
        joints_[i].slider->setValue(newVal);
    }
}

void MainWindow::startGoToPose(const std::array<double, 7>& pose) {
    goalPose_ = pose;
    log("moving to preset pose...");
}

void MainWindow::applyGoToPose() {
    if (!goalPose_) return;
    const double dt = kDtMs / 1000.0;
    bool allReached = true;

    for (size_t i = 0; i < joints_.size() && i < goalPose_->size(); ++i) {
        auto& jp = joints_[i];
        const double target = (*goalPose_)[i];
        const double current = jp.slider->value() / static_cast<double>(kSliderScale);
        const double err = target - current;
        if (std::abs(err) > kGotoToleranceRad) allReached = false;

        const double velLimit = (i < static_cast<size_t>(kNumArmJoints))
            ? ((speedScale_ == SpeedScale::Low) ? kJointVelLow[i] : kJointVelMedium[i])
            : kGotoVelLow;   // gripper slot
        const double maxStep = velLimit * dt;
        const double move = std::clamp(err, -maxStep, maxStep);   // velocity-limited, no overshoot

        const int delta = static_cast<int>(move * kSliderScale);
        int newVal = jp.slider->value() + delta;
        newVal = std::max(jp.slider->minimum(), std::min(jp.slider->maximum(), newVal));
        jp.slider->setValue(newVal);
    }

    if (allReached) {
        goalPose_.reset();
        log("reached preset pose");
    }
}

void MainWindow::onTick() {
    gamepad_.poll();
    controllerStatusLabel_->setText(
        gamepad_.isConnected() ? QString("Controller: %1").arg(gamepad_.name())
                                : "Controller: not connected");

    // Back is a dead-man E-STOP: live in every mode, held = disabled (not
    // just edge-triggered), and cancels any in-progress preset-pose move.
    if (gamepad_.buttonPressed(SDL_CONTROLLER_BUTTON_BACK)) {
        disableAll();
        goalPose_.reset();
    }

    bool dleft = gamepad_.buttonPressed(SDL_CONTROLLER_BUTTON_DPAD_LEFT);
    if (dleft && !prevDpadLeft_) startGoToPose(kPresetPose);
    prevDpadLeft_ = dleft;

    if (goalPose_) {
        applyGoToPose();   // a preset-pose move owns the sliders until it finishes/cancels
    } else if (controlMode_ == ControlMode::TaskSpace) {
        applyTaskSpaceJog();
    } else {
        for (auto& jp : joints_) {
            applyJog(jp, jp.cfg.jog);
        }
    }

    if (!ctrl_) return;

    try {
        ctrl_->poll_feedback_once();
    } catch (const MotorBridgeError& e) {
        log(QString("poll_feedback_once failed: %1").arg(e.what()));
    }

    for (auto& jp : joints_) {
        if (!jp.motor) continue;

        const float targetPos = jp.slider->value() / static_cast<float>(kSliderScale);
        jp.targetLabel->setText(QString("Target: %1 rad").arg(targetPos, 0, 'f', 3));

        if (jp.enabled) {
            bool okKp = false, okKd = false;
            float kp = jp.kpEdit->text().toFloat(&okKp);
            float kd = jp.kdEdit->text().toFloat(&okKd);
            if (okKp && okKd) {
                try {
                    jp.motor->send_mit(targetPos, 0.0f, kp, kd, 0.0f);
                } catch (const MotorBridgeError& e) {
                    jp.statusLabel->setText(QString("send_mit error: %1").arg(e.what()));
                    continue;
                }
            }
        }

        try {
            if (auto st = jp.motor->get_state()) {
                jp.actualLabel->setText(QString("Actual: %1 rad  (vel=%2 torq=%3 t=%4/%5C)")
                    .arg(st->pos, 0, 'f', 3)
                    .arg(st->vel, 0, 'f', 2)
                    .arg(st->torq, 0, 'f', 2)
                    .arg(st->t_mos, 0, 'f', 0)
                    .arg(st->t_rotor, 0, 'f', 0));

                const double err = targetPos - st->pos;
                jp.errorLabel->setText(QString("Err: %1 rad").arg(err, 0, 'f', 3));
                jp.errorLabel->setStyleSheet(std::abs(err) > kPosErrorWarnRad
                                                  ? "color:#ff5555; font-weight:bold;"
                                                  : "color:#5cd65c;");
            }
        } catch (const MotorBridgeError&) {
        }
    }
}
