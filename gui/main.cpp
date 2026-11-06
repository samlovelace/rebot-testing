#include <QApplication>
#include <QPalette>
#include <QColor>
#include <QStyleFactory>

#include "MainWindow.hpp"

namespace {

// Dark, cyan-accented "control console" theme.
void applyDarkTheme(QApplication& app) {
    app.setStyle(QStyleFactory::create("Fusion"));

    const QColor bg(30, 32, 36);
    const QColor bgAlt(40, 43, 48);
    const QColor base(24, 26, 29);
    const QColor text(224, 230, 233);
    const QColor accent(0, 210, 211);     // cyan
    const QColor disabledText(110, 115, 120);

    QPalette pal;
    pal.setColor(QPalette::Window, bg);
    pal.setColor(QPalette::WindowText, text);
    pal.setColor(QPalette::Base, base);
    pal.setColor(QPalette::AlternateBase, bgAlt);
    pal.setColor(QPalette::ToolTipBase, bgAlt);
    pal.setColor(QPalette::ToolTipText, text);
    pal.setColor(QPalette::Text, text);
    pal.setColor(QPalette::Button, bgAlt);
    pal.setColor(QPalette::ButtonText, text);
    pal.setColor(QPalette::BrightText, Qt::red);
    pal.setColor(QPalette::Link, accent);
    pal.setColor(QPalette::Highlight, accent);
    pal.setColor(QPalette::HighlightedText, Qt::black);
    pal.setColor(QPalette::Disabled, QPalette::Text, disabledText);
    pal.setColor(QPalette::Disabled, QPalette::ButtonText, disabledText);
    pal.setColor(QPalette::Disabled, QPalette::WindowText, disabledText);
    app.setPalette(pal);

    app.setStyleSheet(R"(
        QMainWindow, QWidget { background-color: #1e2024; }

        QGroupBox {
            border: 1px solid #3a3f44;
            border-radius: 6px;
            margin-top: 10px;
            padding: 10px 6px 6px 6px;
            font-weight: bold;
            color: #00d2d3;
        }
        QGroupBox::title {
            subcontrol-origin: margin;
            left: 8px;
            padding: 0 4px;
        }

        QPushButton {
            background-color: #2b2f34;
            border: 1px solid #454a50;
            border-radius: 5px;
            padding: 5px 10px;
            color: #e0e6e9;
        }
        QPushButton:hover { border-color: #00d2d3; color: #00d2d3; }
        QPushButton:pressed { background-color: #00d2d3; color: #10141a; }
        QPushButton:disabled { color: #5a5f64; border-color: #333; }

        QLineEdit {
            background-color: #15171a;
            border: 1px solid #3a3f44;
            border-radius: 4px;
            padding: 3px;
            color: #e0e6e9;
            selection-background-color: #00d2d3;
            selection-color: #10141a;
        }
        QLineEdit:focus { border-color: #00d2d3; }

        QSlider::groove:horizontal {
            height: 6px;
            background: #15171a;
            border-radius: 3px;
        }
        QSlider::handle:horizontal {
            background: #00d2d3;
            width: 16px;
            margin: -6px 0;
            border-radius: 8px;
        }
        QSlider::handle:horizontal:hover { background: #4de6e7; }

        QLabel#padLabel {
            color: #9aa0a6;
            font-family: "DejaVu Sans Mono", monospace;
        }
        QLabel#controllerStatus {
            color: #00d2d3;
            font-weight: bold;
        }
        QLabel#jointStatus {
            color: #b7f5f6;
            font-family: "DejaVu Sans Mono", monospace;
        }

        QPlainTextEdit {
            background-color: #121417;
            color: #b7f5f6;
            border: 1px solid #3a3f44;
            font-family: "DejaVu Sans Mono", monospace;
        }

        QScrollArea { border: none; }
    )");
}

} // namespace

int main(int argc, char** argv) {
    QApplication app(argc, argv);
    applyDarkTheme(app);

    MainWindow window;
    window.show();
    return app.exec();
}
