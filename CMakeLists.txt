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
# For CMake < 3.14, add: set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

include(GoogleTest)  # <-- Required for gtest_discover_tests()

find_package(Eigen3 CONFIG REQUIRED)
if (Eigen3_FOUND)
    message(STATUS "Found Eigen3: ${Eigen3_DIR}")
else()
    message(FATAL_ERROR "Eigen3 not found. Please install Eigen3 or adjust CMakeLists.txt to specify its path if vendored.")
endif()

# ------------------- Core Library -------------------
add_library(ekfusion2d_models
    src/models/Unicycle.cpp
)

target_include_directories(ekfusion2d_models PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)
target_link_libraries(ekfusion2d_models PUBLIC Eigen3::Eigen)

# ------------------- Main Executable -------------------
set(MAIN_PROJECT_SOURCES
    src/main.cpp
    src/ekf/ExtendedKalmanFilter.cpp
    src/sim/GroundTruthSimulator.cpp
    src/sim/Sensor.cpp
)

add_executable(ekfusion2d_sim ${MAIN_PROJECT_SOURCES})

target_include_directories(ekfusion2d_sim PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(ekfusion2d_sim PRIVATE
    Eigen3::Eigen
    ekfusion2d_models
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
)

if (TEST_SOURCES)
    add_executable(ekfusion2d_tests ${TEST_SOURCES})

    target_link_libraries(ekfusion2d_tests PRIVATE
        gtest_main
        Eigen3::Eigen
        ekfusion2d_models
    )

    target_include_directories(ekfusion2d_tests PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
    )

    # Automatic discovery of all test cases
    gtest_discover_tests(ekfusion2d_tests
        DISCOVERY_TIMEOUT 10
    )
endif()
