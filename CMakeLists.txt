cmake_minimum_required(VERSION 3.10)

project(test_opencv)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_STAGING_REQUIRED ON)
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")
set(CMAKE_C_FLAGS_DEBUG "-g -O0")

# 查找 OpenCV 库
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
#查找 Egien3库
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_library(ONNXRUNTIME_LIB onnxruntime PATHS /home/liesun/onnxruntime-linux-x64-1.20.1/lib)


# 包含头文件目录
include_directories(./include/camera)
include_directories(./include/armor)
include_directories(./include/thread)
include_directories(./include/tools)
include_directories(/opt/MVS/include)
include_directories(/home/liesun/onnxruntime-linux-x64-1.20.1/include)

# 设置源文件目录
aux_source_directory(src/camera SRC_LIST)
aux_source_directory(src/armor SRC_LIST1)
aux_source_directory(src/thread SRC_LIST2)
aux_source_directory(src/tools SRC_LIST3)

# 汇总所有源文件
set(TOTAL_LIST ${SRC_LIST} ${SRC_LIST1} ${SRC_LIST2} ${SRC_LIST3} src/main.cpp)

# 创建可执行文件
add_executable(test ${TOTAL_LIST})  # 使用 TOTAL_LIST 作为源文件

# 链接 OpenCV 库和 pthread 库

target_link_libraries(test ${OpenCV_LIBS} pthread /opt/MVS/lib/64/libMvCameraControl.so)
target_link_libraries(test ${ONNXRUNTIME_LIB}) 

