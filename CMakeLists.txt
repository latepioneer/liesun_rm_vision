cmake_minimum_required(VERSION 3.10)

project(test_opencv)

# 查找 OpenCV 库
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# 包含头文件目录
include_directories(./include/camera)
include_directories(./include/armor)
include_directories(./include/thread)
include_directories(./include/tools)

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
target_link_libraries(test ${OpenCV_LIBS} pthread)
