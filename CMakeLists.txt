cmake_minimum_required(VERSION 2.8.3)
project(yolo_ros)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE "Release")

find_package(catkin REQUIRED COMPONENTS
message_generation
std_msgs
)

add_message_files(DIRECTORY msg
  FILES 
  DetectionMessage.msg
  DetectionMessages.msg
  )

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
        CATKIN_DEPENDS
        message_runtime
        std_msgs
)
