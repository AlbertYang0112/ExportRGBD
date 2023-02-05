cmake_minimum_required(VERSION 3.6)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

project(exportRGBD)
add_executable(exportRGBD export.cpp)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
find_package(k4a)
target_link_libraries(exportRGBD
    k4a
    k4arecord
    ${OpenCV_LIBS})
