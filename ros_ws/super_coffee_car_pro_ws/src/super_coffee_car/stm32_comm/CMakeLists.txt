cmake_minimum_required(VERSION 3.0.2)
project(stm32_comm)

## 1. 寻找依赖
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  sensor_msgs
  serial
  tf
)

## 2. 声明包信息
catkin_package()

## 3. 头文件路径
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## 4. 编译可执行文件
# 注意：现在文件就在 src 下面，路径非常简单
add_executable(stm32_node src/stm32_node.cpp)

## 5. 链接库
target_link_libraries(stm32_node
  ${catkin_LIBRARIES}
  ${serial_LIBRARIES}
  ${tf_LIBRARIES}
)