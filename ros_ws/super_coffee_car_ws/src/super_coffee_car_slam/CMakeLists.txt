cmake_minimum_required(VERSION 3.0.2)
project(super_coffee_car_slam)

## 寻找依赖包 (根据 package.xml 里的依赖)
find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  sensor_msgs
  nav_msgs
)

## 声明这是一个 catkin 包
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES super_coffee_car_slam
#  CATKIN_DEPENDS rospy geometry_msgs sensor_msgs nav_msgs
#  DEPENDS system_lib
)

## (可选) 安装规则，为了以后 install 方便，现在不写也不影响运行
install(DIRECTORY launch rviz config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)