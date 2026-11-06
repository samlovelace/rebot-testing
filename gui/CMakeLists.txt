find_package(Qt6 COMPONENTS Widgets QUIET)
if(NOT Qt6_FOUND)
    find_package(Qt5 COMPONENTS Widgets REQUIRED)
    set(QT_TARGET Qt5::Widgets)
else()
    set(QT_TARGET Qt6::Widgets)
endif()

find_package(PkgConfig REQUIRED)
pkg_check_modules(SDL2 REQUIRED sdl2)

set(CMAKE_AUTOMOC ON)

add_executable(motor_bridge_gui
    main.cpp
    MainWindow.cpp
    MainWindow.hpp
    Gamepad.cpp
    Gamepad.hpp
)

target_include_directories(motor_bridge_gui PRIVATE ${SDL2_INCLUDE_DIRS})
target_compile_options(motor_bridge_gui PRIVATE ${SDL2_CFLAGS_OTHER})
target_link_libraries(motor_bridge_gui PRIVATE motor_bridge arm_kinematics ${QT_TARGET} ${SDL2_LIBRARIES})
