cmake_minimum_required(VERSION 3.16)
project(stereo_vision VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── Build type ──────────────────────────────────────────────────────────────
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# ── ARM / CM4 optimisation flags ─────────────────────────────────────────────
# -mcpu=cortex-a72     : targets BCM2711 CPU exactly
# -mfpu=neon-fp-armv8  : enables NEON SIMD (vectorises SGBM inner loops)
# -O3                  : aggressive optimisation
# -funroll-loops       : unroll tight loops in disparity computation
# -ffast-math          : allows reordering of FP ops (safe for vision)
# -pipe                : avoid temp files (faster compile on SD card)

set(CM4_FLAGS
    "-mcpu=cortex-a72"
    "-mfpu=neon-fp-armv8"
    "-mfloat-abi=hard"
    "-O3"
    "-funroll-loops"
    "-ffast-math"
    "-pipe"
)

# Apply only when building on/for ARM
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm")
    string(JOIN " " CM4_FLAGS_STR ${CM4_FLAGS})
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} ${CM4_FLAGS_STR}")
    message(STATUS "ARM CM4 optimisations enabled")
else()
    message(STATUS "Non-ARM build — CM4 flags skipped")
endif()

# ── Find packages ─────────────────────────────────────────────────────────────
find_package(OpenCV REQUIRED
    COMPONENTS core imgproc calib3d highgui videoio)

# opencv_ximgproc is optional (WLS filter)
find_package(OpenCV QUIET
    OPTIONAL_COMPONENTS ximgproc)

if(OpenCV_ximgproc_FOUND OR TARGET opencv_ximgproc)
    message(STATUS "opencv-contrib found — WLS filter available (--wls flag)")
    add_compile_definitions(HAVE_XIMGPROC)
else()
    message(STATUS "opencv-contrib not found — WLS filter disabled")
endif()

# libcamera (optional — falls back to V4L2 VideoCapture if not found)
find_package(PkgConfig QUIET)
if(PkgConfig_FOUND)
    pkg_check_modules(LIBCAMERA libcamera)
endif()
if(LIBCAMERA_FOUND)
    message(STATUS "libcamera found — using native capture API")
    add_compile_definitions(HAVE_LIBCAMERA)
else()
    message(STATUS "libcamera not found via pkg-config — using V4L2 VideoCapture")
endif()

# ── Include paths ─────────────────────────────────────────────────────────────
include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
)

if(LIBCAMERA_FOUND)
    include_directories(${LIBCAMERA_INCLUDE_DIRS})
endif()

# ── Targets ───────────────────────────────────────────────────────────────────

# Main stereo pipeline
add_executable(stereo_vision
    src/stereo_vision.cpp
)
target_link_libraries(stereo_vision
    ${OpenCV_LIBS}
    pthread
)
if(LIBCAMERA_FOUND)
    target_link_libraries(stereo_vision ${LIBCAMERA_LIBRARIES})
endif()

# Calibration tool
add_executable(calibrate
    src/calibrate.cpp
)
target_link_libraries(calibrate
    ${OpenCV_LIBS}
    pthread
)

# Depth query / click-to-measure utility
add_executable(depth_query
    src/depth_query.cpp
)
target_link_libraries(depth_query
    ${OpenCV_LIBS}
    pthread
)

# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS stereo_vision calibrate depth_query
    RUNTIME DESTINATION bin)

# ── Print summary ─────────────────────────────────────────────────────────────
message(STATUS "")
message(STATUS "=== Stereo Vision Build Summary ===")
message(STATUS "  Build type   : ${CMAKE_BUILD_TYPE}")
message(STATUS "  OpenCV       : ${OpenCV_VERSION}")
message(STATUS "  libcamera    : ${LIBCAMERA_FOUND}")
message(STATUS "  WLS filter   : ${OpenCV_ximgproc_FOUND}")
message(STATUS "  Install to   : ${CMAKE_INSTALL_PREFIX}/bin")
message(STATUS "")
