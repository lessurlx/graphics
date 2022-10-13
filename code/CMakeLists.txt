cmake_minimum_required(VERSION 3.21)
project(code)

set(CMAKE_CXX_STANDARD 14)

include_directories(./Lib)

find_package(OpenCV)

add_executable(Transformation Transformation/Transformation.cpp)
add_executable(Rasterizer Rasterizer/main.cpp Rasterizer/rasterizer.hpp Rasterizer/rasterizer.cpp Rasterizer/Triangle.hpp Rasterizer/Triangle.cpp)