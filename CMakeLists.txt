cmake_minimum_required(VERSION 3.16)
project(motor_bridge CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT DEFINED MOTOR_ABI_LIB)
    # Ask the actual python3 on PATH where it put motorbridge's bundled .so,
    # instead of hardcoding a python version/site-packages layout -- this is
    # what lets the same CMakeLists work unmodified across machines (e.g. an
    # x86_64 dev box on Python 3.10 vs a Jetson that might ship a different
    # default python3).
    find_package(Python3 COMPONENTS Interpreter REQUIRED)
    execute_process(
        COMMAND ${Python3_EXECUTABLE} -c
                "import motorbridge, os; print(os.path.join(os.path.dirname(motorbridge.__file__), 'lib', 'libmotor_abi.so'))"
        OUTPUT_VARIABLE MOTOR_ABI_LIB
        OUTPUT_STRIP_TRAILING_WHITESPACE
        RESULT_VARIABLE MOTOR_ABI_LIB_RESULT
    )
    if(NOT MOTOR_ABI_LIB_RESULT EQUAL 0 OR NOT EXISTS "${MOTOR_ABI_LIB}")
        message(FATAL_ERROR
            "Could not locate libmotor_abi.so via '${Python3_EXECUTABLE} -c \"import motorbridge\"'. "
            "Install it first: pip3 install --user motorbridge (see setup.sh), "
            "or pass -DMOTOR_ABI_LIB=/path/to/libmotor_abi.so explicitly.")
    endif()
    message(STATUS "Found motorbridge ABI library: ${MOTOR_ABI_LIB}")
endif()

add_library(motor_bridge STATIC src/motor_bridge.cpp)
target_include_directories(motor_bridge PUBLIC include)
target_link_libraries(motor_bridge PUBLIC ${MOTOR_ABI_LIB})

add_executable(motor_bridge_test examples/test_main.cpp)
target_link_libraries(motor_bridge_test PRIVATE motor_bridge)

find_package(Eigen3 REQUIRED NO_MODULE)

add_library(arm_kinematics STATIC gui/ArmKinematics.cpp)
target_include_directories(arm_kinematics PUBLIC gui)
target_link_libraries(arm_kinematics PUBLIC Eigen3::Eigen)

add_executable(test_kinematics examples/test_kinematics.cpp)
target_link_libraries(test_kinematics PRIVATE arm_kinematics)

add_subdirectory(gui)
