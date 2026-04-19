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

# OpenCV (OPTIONAL - enables real-camera vision tracker)
# Hint locations checked in order:
#   1. env OpenCV_DIR (explicit user override)
#   2. vcpkg installed paths (x64-windows, x64-windows-static)
#   3. opencv.org self-extracted prebuilt at C:/opencv/build
#   4. external/opencv/build (project-local bundle)
#
# vcpkg packages declare transitive find_dependency() calls (e.g. Protobuf)
# that only resolve when their root prefix is on CMAKE_PREFIX_PATH. Detect
# the vcpkg installed root and prepend it before find_package(OpenCV).
set(_uav_vcpkg_root "")
foreach(_root IN ITEMS
        "$ENV{VCPKG_ROOT}/installed/x64-windows"
        "C:/vcpkg/installed/x64-windows"
        "C:/dev/vcpkg/installed/x64-windows")
    if(EXISTS "${_root}/share/opencv4/OpenCVConfig.cmake")
        set(_uav_vcpkg_root "${_root}")
        break()
    endif()
endforeach()
if(_uav_vcpkg_root)
    list(PREPEND CMAKE_PREFIX_PATH "${_uav_vcpkg_root}")
    message(STATUS "vcpkg installed root added to CMAKE_PREFIX_PATH: ${_uav_vcpkg_root}")
endif()

set(_uav_opencv_hints
    "$ENV{OpenCV_DIR}"
    "$ENV{VCPKG_ROOT}/installed/x64-windows/share/opencv4"
    "C:/vcpkg/installed/x64-windows/share/opencv4"
    "C:/vcpkg/installed/x64-windows-static/share/opencv4"
    "C:/opencv/build"
    "C:/opencv/build/x64/vc16/lib"
    "C:/opencv/build/x64/vc17/lib"
    "C:/Program Files/opencv/build"
    "${CMAKE_SOURCE_DIR}/external/opencv/build"
)
find_package(OpenCV QUIET COMPONENTS core imgproc videoio highgui imgcodecs
             HINTS ${_uav_opencv_hints})
if(OpenCV_FOUND)
    message(STATUS "OpenCV found: version ${OpenCV_VERSION} at ${OpenCV_DIR}")
    set(UAV_HAS_OPENCV ON)
    # Resolve runtime DLL directory so we can stage required DLLs next to
    # uav_system.exe at build time (avoids 'DLL not found' on launch without
    # touching PATH). Layouts handled:
    #   vcpkg     : OpenCV_DIR=...\installed\x64-windows\share\opencv4 -> ..\..\bin
    #   prebuilt  : OpenCV_DIR=...\build (or build\x64\vc16\lib) -> build\x64\vcXX\bin
    get_filename_component(_uav_opencv_p1 "${OpenCV_DIR}/.."        ABSOLUTE)
    get_filename_component(_uav_opencv_p2 "${OpenCV_DIR}/../.."     ABSOLUTE)
    get_filename_component(_uav_opencv_p3 "${OpenCV_DIR}/../../.."  ABSOLUTE)
    set(_uav_opencv_bin_candidates
        "${_uav_opencv_p2}/bin"          # vcpkg / unix-style
        "${_uav_opencv_p1}/bin"
        "${_uav_opencv_p3}/bin"
        "${_uav_opencv_p1}/x64/vc17/bin" # opencv.org prebuilt
        "${_uav_opencv_p1}/x64/vc16/bin"
        "${_uav_opencv_p1}/x64/vc15/bin"
        "${_uav_opencv_p2}/x64/vc17/bin"
        "${_uav_opencv_p2}/x64/vc16/bin"
        "${_uav_opencv_p3}/x64/vc17/bin"
        "${_uav_opencv_p3}/x64/vc16/bin"
        "${_uav_vcpkg_root}/bin"         # explicit vcpkg fallback
    )
    set(UAV_OPENCV_BIN_DIR "")
    foreach(_d IN LISTS _uav_opencv_bin_candidates)
        if(_d AND EXISTS "${_d}/opencv_core4.dll" OR _d AND EXISTS "${_d}/opencv_core470.dll" OR _d AND EXISTS "${_d}/opencv_core411.dll")
            set(UAV_OPENCV_BIN_DIR "${_d}")
            break()
        endif()
    endforeach()
    # Last-resort: any opencv_*.dll in any candidate dir
    if(NOT UAV_OPENCV_BIN_DIR)
        foreach(_d IN LISTS _uav_opencv_bin_candidates)
            if(_d AND EXISTS "${_d}")
                file(GLOB _uav_probe "${_d}/opencv_*.dll")
                if(_uav_probe)
                    set(UAV_OPENCV_BIN_DIR "${_d}")
                    break()
                endif()
            endif()
        endforeach()
    endif()
    if(UAV_OPENCV_BIN_DIR)
        message(STATUS "OpenCV runtime DLLs: ${UAV_OPENCV_BIN_DIR}")
    else()
        message(STATUS "OpenCV runtime DLL dir not auto-located - add to PATH manually if needed.")
    endif()
else()
    message(STATUS "OpenCV NOT found - real-camera vision tracker disabled.")
    message(STATUS "  Tried HINTS: ${_uav_opencv_hints}")
    message(STATUS "  To enable, install OpenCV (e.g. vcpkg install opencv4) and re-run CMake")
    message(STATUS "  with -DOpenCV_DIR=<path-to-OpenCVConfig.cmake> if auto-detect fails.")
    set(UAV_HAS_OPENCV OFF)
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
    src/perception/perception_engine.cpp
    src/perception/simple_vision_tracker.cpp
    src/perception/simple_qr_detector.cpp
    src/perception/camera/opencv_camera.cpp
    src/perception/detector/round_detector.cpp
    src/perception/opencv_vision_tracker.cpp
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

# Enable spdlog DEBUG level macros (SPDLOG_DEBUG/LOG_DEBUG)
# Without this, all SPDLOG_DEBUG calls compile to no-op regardless of runtime level.
target_compile_definitions(uav_lib PUBLIC SPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_DEBUG)

# OpenCV link + define so source-level #ifdef UAV_HAS_OPENCV gates compile in
if(UAV_HAS_OPENCV)
    target_compile_definitions(uav_lib PUBLIC UAV_HAS_OPENCV)
    target_include_directories(uav_lib PUBLIC ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(uav_lib PUBLIC ${OpenCV_LIBS})
    # OpenCV headers in unity builds occasionally hit MSVC C2872 (typename
    # ambiguity); keep the camera/detector TUs out of the unity batch.
    set_source_files_properties(
        src/perception/camera/opencv_camera.cpp
        src/perception/detector/round_detector.cpp
        src/perception/opencv_vision_tracker.cpp
        PROPERTIES SKIP_UNITY_BUILD_INCLUSION ON)
endif()

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

# Stage OpenCV runtime DLLs next to uav_system.exe so it launches without
# requiring the user to extend PATH. Skipped silently when bin dir was not
# auto-located (user can still adjust PATH).
if(UAV_HAS_OPENCV AND WIN32 AND UAV_OPENCV_BIN_DIR)
    file(GLOB UAV_OPENCV_DLLS "${UAV_OPENCV_BIN_DIR}/opencv_*.dll")
    # vcpkg also ships transitive deps in the same folder (libpng, jpeg,
    # zlib, webp, ...). Mirror them too so videoio/imgcodecs work out of
    # the box.
    file(GLOB UAV_OPENCV_EXTRA_DLLS
        "${UAV_OPENCV_BIN_DIR}/zlib*.dll"
        "${UAV_OPENCV_BIN_DIR}/libpng*.dll"
        "${UAV_OPENCV_BIN_DIR}/libjpeg*.dll"
        "${UAV_OPENCV_BIN_DIR}/jpeg*.dll"
        "${UAV_OPENCV_BIN_DIR}/turbojpeg*.dll"
        "${UAV_OPENCV_BIN_DIR}/libwebp*.dll"
        "${UAV_OPENCV_BIN_DIR}/libsharpyuv*.dll"
        "${UAV_OPENCV_BIN_DIR}/tiff*.dll"
        "${UAV_OPENCV_BIN_DIR}/liblzma*.dll"
        "${UAV_OPENCV_BIN_DIR}/lzma*.dll"
        "${UAV_OPENCV_BIN_DIR}/libprotobuf*.dll"
        "${UAV_OPENCV_BIN_DIR}/abseil*.dll"
    )
    list(APPEND UAV_OPENCV_DLLS ${UAV_OPENCV_EXTRA_DLLS})
    if(UAV_OPENCV_DLLS)
        add_custom_command(TARGET uav_system POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                ${UAV_OPENCV_DLLS}
                $<TARGET_FILE_DIR:uav_system>
            COMMENT "Staging OpenCV runtime DLLs next to uav_system.exe")
    endif()
endif()

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
message(STATUS "OpenCV vision: ${UAV_HAS_OPENCV}")
message(STATUS "=============================")
message(STATUS "")
