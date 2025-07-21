cmake_minimum_required(VERSION 3.10)

project(EKFusion2D CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/release-1.12.1.zip 
)
FetchContent_MakeAvailable(googletest)

find_package(Eigen3 CONFIG REQUIRED)
if (Eigen3_FOUND)
    message(STATUS "Found Eigen3: ${Eigen3_DIR}")
else()
    message(FATAL_ERROR "Eigen3 not found. Please install Eigen3 or adjust CMakeLists.txt to specify its path if vendored.")
endif()

set(MAIN_PROJECT_SOURCES
    src/main.cpp
    src/ekf/ExtendedKalmanFilter.cpp
    src/models/UnicycleModel.cpp
    src/models/SensorModels.cpp
    src/sim/GroundTruthSimulator.cpp
    src/sim/NoisySensorSimulator.cpp
)

add_executable(ekfusion2d_sim ${MAIN_PROJECT_SOURCES})

target_include_directories(ekfusion2d_sim PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(ekfusion2d_sim PRIVATE Eigen3::Eigen)

if (CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang") 
    target_compile_options(ekfusion2d_sim PRIVATE
        -Wall
        -Wextra
        -Wpedantic
        -Werror
    )
endif()


set(TEST_SOURCES
    test/test_unicycle_model.cpp 
)

if (TEST_SOURCES)
    add_executable(ekfusion2d_tests ${TEST_SOURCES})
    target_link_libraries(ekfusion2d_tests PRIVATE
        gtest_main
        gtest
    )

    target_include_directories(ekfusion2d_tests PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
    )

    include(GoogleTest)
    gtest_discover_tests(ekfusion2d_tests)
endif()