cmake_minimum_required(VERSION 2.8.3)
project(mono_slam_tensorflow)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  #cv_bridge
  #image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  #message_generation
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
#find_package (OpenCV REQUIRED)




## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   camera.msg
 #)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   sensor_msgs
   std_msgs
 )

# catkin_package(

#   CATKIN_DEPENDS message_runtime
#   DEPENDS OpenCV
#   )

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Specify libraries to link a library or executable target against
#  target_link_libraries(${PROJECT_NAME}
#   ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}
 #)
