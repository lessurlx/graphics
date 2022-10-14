cmake_minimum_required(VERSION 3.21)
project(code)

set(CMAKE_CXX_STANDARD 14)
include_directories(./Lib)
add_executable(Transformation Transformation/Transformation.cpp)

# 搜索OpenCV目录
set(OpenCV_DIR "D:\\opencv\\build_mingw64\\install")
find_package(OpenCV REQUIRED)

# 添加OpenCV头文件目录
include_directories(${OpenCV_INCLUDE_DIRS})

# 链接OpenCV库文件
add_executable(Rasterizer Rasterizer/main.cpp Rasterizer/rasterizer.hpp Rasterizer/rasterizer.cpp Rasterizer/Triangle.hpp Rasterizer/Triangle.cpp)
target_link_libraries(Rasterizer ${OpenCV_LIBS})
