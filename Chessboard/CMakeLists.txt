cmake_minimum_required(VERSION 3.16)
project(CameraCalibration)

set(CMAKE_CXX_STANDARD 14)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(CameraCalibration main.cpp CameraCalibrator.cpp CameraCalibrator.hpp)
target_link_libraries(CameraCalibration PRIVATE ${OpenCV_LIBS} OpenCL::Headers OpenCL::OpenCL)
