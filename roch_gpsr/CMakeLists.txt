cmake_minimum_required(VERSION 2.8.3)
project(roch_gpsr)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  smach
  smach_ros
  actionlib
  actionlib_msgs
  sensor_msgs
  pcl_ros
  pcl_conversions
  image_transport
  cv_bridge
  dynamic_reconfigure
)

find_package(PCL 1.1 REQUIRED)
find_package(OpenCV REQUIRED)

add_action_files(
  DIRECTORY
    action    
  FILES
    #iARM.action
    #ObjRec.action
    #SpeechSyn.action
    TaskRec.action
    SoundEffect.action
)

generate_messages(
  DEPENDENCIES
    actionlib_msgs
    std_msgs
    sensor_msgs 
)


generate_dynamic_reconfigure_options(
  cfg/ObjDetect.cfg
  #...
)

# add_library(roch_gpsr
#    scripts/common/common_function.py
# )



catkin_package(
  DEPENDS
  CATKIN_DEPENDS
  actionlib_msgs
  dynamic_reconfigure
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

# PCL
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# OpenCV
include_directories(${OpenCV_INCLUDE_DIRS})


add_executable(task_rec_server src/task_rec_server.cpp)
target_link_libraries(task_rec_server 
  ${catkin_LIBRARIES}
  /usr/lib/x86_64-linux-gnu/libtinyxml2.a
)

add_executable(obj_detect_server
  src/obj_detect_server.cpp 
  src/common_function.cpp
)
add_dependencies(obj_detect_server ${PROJECT_NAME}_gencfg)
target_link_libraries(obj_detect_server 
  ${catkin_LIBRARIES}
)


add_executable(obj_detect_test src/obj_detect_test.cpp)
target_link_libraries(obj_detect_test 
  ${catkin_LIBRARIES}
)




