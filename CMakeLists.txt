cmake_minimum_required(VERSION 3.20)

project(
    antagonistic_joint_sim2real
    VERSION 0.1.0
    LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_library(
    atj_model
    src/joint_model.cpp
    src/integrator.cpp
)

target_include_directories(
    atj_model
    PUBLIC include
)

target_compile_options(
    atj_model
    PRIVATE
        -Wall
        -Wextra
        -Wpedantic
        -Wconversion
)

enable_testing()

add_executable(
    test_joint_model
    tests/test_joint_model.cpp
)

target_link_libraries(
    test_joint_model
    PRIVATE atj_model
)

add_test(
    NAME joint_model
    COMMAND test_joint_model
)

add_executable(
    test_integrator
    tests/test_integrator.cpp
)

target_link_libraries(
    test_integrator
    PRIVATE atj_model
)

add_test(
    NAME integrator
    COMMAND test_integrator
)

add_executable(
    simulate_step
    apps/simulate_step.cpp
)

target_link_libraries(
    simulate_step
    PRIVATE atj_model
)
