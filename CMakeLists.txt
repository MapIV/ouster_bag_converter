cmake_minimum_required(VERSION 3.0)
project(ouster_bag_converter)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  rosbag
)

find_package(PCL REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS sensor_msgs rosbag
  DEPENDS PCL
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(ouster_bag_converter
  src/ouster_bag_converter.cpp
)
target_link_libraries(ouster_bag_converter
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
