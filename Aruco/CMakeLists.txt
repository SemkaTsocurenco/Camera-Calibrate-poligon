cmake_minimum_required(VERSION 3.16)
project(CameraChessBoardCalibration)

set(CMAKE_CXX_STANDARD 17)
set(EXECUTABLE_OUTPUT_PATH ".")
set(CMAKE_CXX_FLAGS "-fPIC -O3 -ffast-math -msse3 -mfpmath=sse")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/modules/")
set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
find_package(Threads REQUIRED)


find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(CameraChessBoardCalibration main.cpp CameraCalibrator.cpp CameraCalibrator.h)
string(CONCAT CMAKE_PREFIX_PATH "/home/tsokurenkosv/booba_test/buba/CL/OpenCL-Headers/install;" "/home/tsokurenkosv/booba_test/buba/CL/OpenCL-ICD-Loader/install/")

target_link_libraries(CameraChessBoardCalibration Threads::Threads)
target_link_libraries(CameraChessBoardCalibration ${OpenCV_LIBS} )

