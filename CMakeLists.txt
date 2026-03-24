cmake_minimum_required(VERSION 3.10)
project(stereo_vision)

find_package(OpenCV REQUIRED)

add_executable(stereo src/stereo.cpp)

target_link_libraries(stereo ${OpenCV_LIBS})