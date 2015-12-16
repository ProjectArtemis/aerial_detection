cmake_minimum_required(VERSION 2.8.3)
project(aerial_detection)

# CPU-only Caffe
add_definitions(-DCPU_ONLY=1)

find_package(catkin REQUIRED COMPONENTS
  roscpp roslib std_msgs sensor_msgs image_transport cv_bridge message_generation
)

set(CAFFE_INCLUDEDIR caffe/include caffe/distribute/include)
set(CAFFE_LINK_LIBRARAY caffe/build/lib)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV REQUIRED)

add_message_files(
  DIRECTORY msg
  FILES
  detections.msg
)

generate_messages(DEPENDENCIES std_msgs)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
include_directories(${CAFFE_INCLUDEDIR})

link_directories(${CAFFE_LINK_LIBRARAY})
add_executable(detector_node src/classifier.cpp)
target_link_libraries(detector_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} caffe glog)
