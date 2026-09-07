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

include(FetchContent)

set(
    FETCHCONTENT_TRY_FIND_PACKAGE_MODE
    NEVER
)

FetchContent_Declare(
    Catch2
    GIT_REPOSITORY https://github.com/catchorg/Catch2.git
    GIT_TAG v3.15.3
    GIT_SHALLOW TRUE
)

FetchContent_MakeAvailable(Catch2)

list(
    APPEND CMAKE_MODULE_PATH
    ${catch2_SOURCE_DIR}/extras
)

include(CTest)
include(Catch)

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

add_executable(
    atj_tests
    tests/test_joint_model.cpp
    tests/test_integrator.cpp
)

target_link_libraries(
    atj_tests
    PRIVATE 
        atj_model
        Catch2::Catch2WithMain
)

target_compile_options(
    atj_tests
    PRIVATE
        -Wall
        -Wextra
        -Wpedantic
        -Wconversion
)

catch_discover_tests(atj_tests)

add_executable(
    simulate_step
    apps/simulate_step.cpp
)

target_link_libraries(
    simulate_step
    PRIVATE atj_model
)

target_compile_options(
    simulate_step
    PRIVATE
        -Wall
        -Wextra
        -Wpedantic
        -Wconversion
)

add_executable(
    validate_open_loop
    apps/validate_open_loop.cpp
)

target_link_libraries(
    validate_open_loop
    PRIVATE atj_model
)

target_compile_options(
    validate_open_loop
    PRIVATE
        -Wall
        -Wextra
        -Wpedantic
        -Wconversion
)

add_executable(
    sweep_inertia
    apps/sweep_inertia.cpp
)

target_link_libraries(
    sweep_inertia
    PRIVATE atj_model
)

target_compile_options(
    sweep_inertia
    PRIVATE
        -Wall
        -Wextra
        -Wpedantic
        -Wconversion
)
