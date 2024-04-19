
project(UDP)

cmake_minimum_required(VERSION 3.16)

find_package(OpenCLHeaders REQUIRED)
find_package(OpenCLICDLoader REQUIRED)

add_subdirectory(Aruco)
add_subdirectory(Chessboard)



