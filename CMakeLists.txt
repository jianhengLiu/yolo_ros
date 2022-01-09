# CMake lowest version requirement
cmake_minimum_required(VERSION 3.5.1)

# project information
project(yolo_ros)


# Compile options
add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE "Release")
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "../out")
set(CMAKE_CXX_FLAGS_DEBUG "-fPIC -O0 -g -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "-fPIC -O2 -Wall")

set(INC_PATH $ENV{DDK_PATH})

if (NOT DEFINED ENV{DDK_PATH})
    set(INC_PATH "/usr/local/Ascend")
    message(STATUS "set default INC_PATH: ${INC_PATH}")
else ()
    message(STATUS "env INC_PATH: ${INC_PATH}")
endif ()

set(LIB_PATH $ENV{NPU_HOST_LIB})

if (NOT DEFINED ENV{NPU_HOST_LIB})
    set(LIB_PATH "/usr/local/Ascend/acllib/lib64/stub/")
    message(STATUS "set default LIB_PATH: ${LIB_PATH}")
else ()
    message(STATUS "env LIB_PATH: ${LIB_PATH}")
endif ()

#if (${CMAKE_CXX_COMPILER} MATCHES "aarch")
set(ARCH "arm")
#else ()
#    set(ARCH "x86")
#endif ()

find_package(catkin REQUIRED COMPONENTS
        cv_bridge
        roscpp
        std_msgs
        roslib
        nodelet
        message_generation
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

# Header path
include_directories(
        $ENV{HOME}/ascend_ddk/${ARCH}/include/opencv4
        $ENV{HOME}/ascend_ddk/${ARCH}/include/
        ${INC_PATH}/acllib/include/
        ./inc/
        ${catkin_INCLUDE_DIRS}
)

if (target STREQUAL "Simulator_Function")
    add_compile_options(-DFUNC_SIM)
endif ()

# add host lib path
link_directories(
        /usr/local/lib
        ${LIB_PATH}
        $ENV{HOME}/ascend_ddk/${ARCH}/lib/
        $ENV{HOME}/Ascend/
        ${INC_PATH}/atc/lib64
)

add_library(yolo_atlas_nodelet
        src/yolo_atlas_nodelet.cpp
        src/utils.cpp
        src/model_process.cpp
        src/object_detect_ros.cpp
        )
target_link_libraries(yolo_atlas_nodelet
        ascendcl
        acl_dvpp
        stdc++
        ${catkin_LIBRARIES}
        )
add_dependencies(yolo_atlas_nodelet yolo_ros_generate_messages_cpp)