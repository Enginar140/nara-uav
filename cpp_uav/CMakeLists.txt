cmake_minimum_required(VERSION 3.16)
project(UAV_V10 VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# =============================================================================
# Compiler flags (cross-platform)
# =============================================================================
if(MSVC)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W4 /utf-8")
    set(CMAKE_CXX_FLAGS_DEBUG "/Zi /Od /DEBUG")
    set(CMAKE_CXX_FLAGS_RELEASE "/O2 /DNDEBUG")
    add_compile_definitions(NOMINMAX WIN32_LEAN_AND_MEAN _CRT_SECURE_NO_WARNINGS)
else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
    set(CMAKE_CXX_FLAGS_DEBUG "-g -O0 -DDEBUG")
    set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")
endif()

set(CMAKE_UNITY_BUILD ON)
set(CMAKE_UNITY_BUILD_BATCH_SIZE 16)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# =============================================================================
# Platform detection
# =============================================================================
find_package(Threads REQUIRED)

# =============================================================================
# External Dependencies (bundled)
# =============================================================================

# Eigen3
set(EIGEN3_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/external/eigen-3.4.0)
if(EXISTS ${EIGEN3_INCLUDE_DIR})
    message(STATUS "Eigen3 found at: ${EIGEN3_INCLUDE_DIR}")
else()
    message(WARNING "Eigen3 not found (optional)")
endif()

# spdlog (header-only)
set(SPDLOG_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/external/spdlog-1.12.0/include)
if(EXISTS ${SPDLOG_INCLUDE_DIR})
    message(STATUS "spdlog found at: ${SPDLOG_INCLUDE_DIR}")
else()
    message(FATAL_ERROR "spdlog not found in external/spdlog-1.12.0")
endif()

# yaml-cpp (build from source)
set(YAML_CPP_DIR ${CMAKE_SOURCE_DIR}/external/yaml-cpp-0.8.0)
if(EXISTS ${YAML_CPP_DIR})
    message(STATUS "yaml-cpp found at: ${YAML_CPP_DIR}")
    file(GLOB YAML_CPP_SOURCES "${YAML_CPP_DIR}/src/*.cpp")
    add_library(yaml_cpp_static STATIC ${YAML_CPP_SOURCES})
    target_include_directories(yaml_cpp_static PUBLIC ${YAML_CPP_DIR}/include)
    target_compile_definitions(yaml_cpp_static PUBLIC YAML_CPP_STATIC_DEFINE)
    set_target_properties(yaml_cpp_static PROPERTIES UNITY_BUILD OFF)
else()
    message(FATAL_ERROR "yaml-cpp not found in external/yaml-cpp-0.8.0")
endif()

# nlohmann_json (header-only)
set(NLOHMANN_JSON_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/external)

# MAVLink (header-only)
set(MAVLINK_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/external/mavlink)
if(EXISTS ${MAVLINK_INCLUDE_DIR}/common/mavlink.h OR EXISTS ${MAVLINK_INCLUDE_DIR}/mavlink_types.h)
    message(STATUS "MAVLink found at: ${MAVLINK_INCLUDE_DIR}")
else()
    message(WARNING "MAVLink not found, comm module will not compile")
endif()

# =============================================================================
# UAV Library
# =============================================================================

set(UAV_SOURCES
    src/core/types.cpp
    src/core/geo.cpp
    src/config/config_manager.cpp
    src/infra/log_manager.cpp
    src/infra/event_bus.cpp
    src/infra/thread_pool.cpp
    src/world_model/world_model.cpp
    src/safety/zone_checker.cpp
    src/safety/safety_gate.cpp
    src/safety/safety_monitor.cpp
    src/safety/escape_manager.cpp
    src/nav/path_planner.cpp
    src/nav/waypoint_chain.cpp
    src/nav/navigation_engine.cpp
    src/comm/flight_commander.cpp
    src/comm/connection_manager.cpp
    src/comm/statustext_parser.cpp
    src/comm/param_reader.cpp
    src/comm/mavlink_bridge.cpp
    src/mission/patrol_mode.cpp
    src/mission/follow_mode.cpp
    src/mission/qr_dive_mode.cpp
    src/mission/ardupilot_mode.cpp
    src/mission/mission_director.cpp
    src/vision/tracking_controller.cpp
    src/app/application.cpp
)

add_library(uav_lib STATIC ${UAV_SOURCES})

target_include_directories(uav_lib PUBLIC
    ${CMAKE_SOURCE_DIR}/include
    ${MAVLINK_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIR}
    ${SPDLOG_INCLUDE_DIR}
    ${NLOHMANN_JSON_INCLUDE_DIR}
    ${YAML_CPP_DIR}/include
)

target_link_libraries(uav_lib PUBLIC
    yaml_cpp_static
    Threads::Threads
)

# Platform-specific link libraries
if(WIN32)
    target_link_libraries(uav_lib PUBLIC ws2_32)
endif()

# GCC < 9 needs explicit stdc++fs for std::filesystem
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 9.0)
    target_link_libraries(uav_lib PUBLIC stdc++fs)
endif()

# Clang may need c++fs
if(CMAKE_CXX_COMPILER_ID STREQUAL "Clang" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 9.0)
    target_link_libraries(uav_lib PUBLIC c++fs)
endif()

# =============================================================================
# Executable
# =============================================================================

add_executable(uav_system src/main.cpp)
target_link_libraries(uav_system PRIVATE uav_lib)

# =============================================================================
# Copy config
# =============================================================================

file(COPY config/ DESTINATION ${CMAKE_BINARY_DIR}/config/)

# =============================================================================
# Summary
# =============================================================================

message(STATUS "")
message(STATUS "=== UAV v10 Configuration ===")
message(STATUS "Build type:    ${CMAKE_BUILD_TYPE}")
message(STATUS "C++ standard:  ${CMAKE_CXX_STANDARD}")
message(STATUS "Compiler:      ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")
message(STATUS "System:        ${CMAKE_SYSTEM_NAME} ${CMAKE_SYSTEM_PROCESSOR}")
message(STATUS "=============================")
message(STATUS "")
