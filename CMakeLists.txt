cmake_minimum_required(VERSION 3.10)

project(EKFusion2D CXX)

enable_testing()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

cmake_policy(SET CMP0135 NEW)

include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/release-1.12.1.zip
)
FetchContent_MakeAvailable(googletest)

include(GoogleTest)

find_package(Eigen3 CONFIG REQUIRED)
if (Eigen3_FOUND)
    message(STATUS "Found Eigen3: ${Eigen3_DIR}")
else()
    message(FATAL_ERROR "Eigen3 not found. Please install Eigen3 or adjust CMakeLists.txt to specify its path if vendored.")
endif()

# ------------------- Core Library -------------------
# A single library for all core, reusable components
add_library(ekfusion2d_lib
    src/models/Unicycle.cpp
    src/ekf/ExtendedKalmanFilter.cpp
    src/sim/GroundTruthSimulator.cpp
    src/sim/Sensor.cpp
    src/common/NoiseGenerator.cpp
)

target_include_directories(ekfusion2d_lib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)
target_link_libraries(ekfusion2d_lib PUBLIC Eigen3::Eigen)

# ------------------- Main Executable -------------------
add_executable(ekfusion2d_sim
    src/main.cpp
)

target_include_directories(ekfusion2d_sim PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(ekfusion2d_sim PRIVATE
    ekfusion2d_lib
)

if (CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(ekfusion2d_sim PRIVATE
        -Wall
        -Wextra
        -Wpedantic
        -Werror
    )
endif()

# ------------------- Tests -------------------
set(TEST_SOURCES
    tests/test_unicycle_model.cpp
    tests/test_generate_correlated_noise.cpp
    tests/test_ground_truth_simulator.cpp
    tests/test_helper.hpp
)

if (TEST_SOURCES)
    add_executable(ekfusion2d_tests ${TEST_SOURCES})

    target_link_libraries(ekfusion2d_tests PRIVATE
        gtest_main
        gmock
        Eigen3::Eigen
        ekfusion2d_lib
    )

    target_include_directories(ekfusion2d_tests PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${gtest_SOURCE_DIR}/googlemock/include
    )

    gtest_discover_tests(ekfusion2d_tests
        DISCOVERY_TIMEOUT 10
    )
endif()