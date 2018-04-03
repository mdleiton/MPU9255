cmake_minimum_required(VERSION 2.8.3)
project(MPU9255)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  rosbag
  tf
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(MPU9255 src/MPU9255_node.cpp)
target_link_libraries(MPU9255 ${catkin_LIBRARIES})
target_link_libraries (MPU9255 wiringPi)

