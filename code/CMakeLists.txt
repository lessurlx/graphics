cmake_minimum_required(VERSION 3.21)
project(code)

set(CMAKE_CXX_STANDARD 14)

include_directories(./Lib)

add_executable(code main.cpp)
add_executable(Transformation Transformation.cpp)
