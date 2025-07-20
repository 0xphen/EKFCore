cmake_minimum_required(VERSION 3.10)
project(EKFusion2D CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(PROJECT_SOURCES
    src/main.cpp
    src/ekf/ExtendedKalmanFilter.cpp
    src/models/VehicleModel.cpp
    src/models/SensorModels.cpp
    src/sim/GroundTruthSimulator.cpp
    src/sim/NoisySensorSimulator.cpp
)

add_executable(ekfusion2d_sim ${PROJECT_SOURCES})

target_include_directories(ekfusion2d_sim PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

find_package(Eigen3 CONFIG REQUIRED)
if (Eigen3_FOUND)
    message(STATUS "Found Eigen3: ${Eigen3_DIR}") # Good for debugging
    target_link_libraries(ekfusion2d_sim PRIVATE Eigen3::Eigen)
else()
    message(FATAL_ERROR "Eigen3 not found. Please install Eigen3 or adjust CMakeLists.txt to specify its path if vendored.")
endif()

if (CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(ekfusion2d_sim PRIVATE
        -Wall          # All common warnings
        -Wextra        # Extra warnings
        -Wpedantic     # Strict ISO C++ compliance
        -Werror        # Treat warnings as errors (good for CI/CD)
    )
endif()