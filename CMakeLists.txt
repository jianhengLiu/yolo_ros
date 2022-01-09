cmake_minimum_required(VERSION 2.8.3)
project(yolo_ros)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE "Release")

find_package(catkin REQUIRED COMPONENTS
message_generation
std_msgs
roscpp
sensor_msgs
cv_bridge
actionlib
actionlib_msgs
)

find_package(OpenCV REQUIRED)

add_action_files(DIRECTORY action
   FILES
   yolo.action
)

add_service_files(DIRECTORY srv
  FILES
  yolo.srv
)

add_message_files(DIRECTORY msg
  FILES 
  DetectionMessage.msg
  DetectionMessages.msg
  bbox2d.msg
  result.msg
  )

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  actionlib_msgs
)

catkin_package(
        CATKIN_DEPENDS
        message_runtime
        std_msgs
        sensor_msgs
        actionlib_msgs
)

include_directories(
        ${catkin_INCLUDE_DIRS}
)

## target
add_executable(service_client src/service_client.cpp)
target_link_libraries(service_client ${catkin_LIBRARIES} ${OpenCV_LIBS} )
