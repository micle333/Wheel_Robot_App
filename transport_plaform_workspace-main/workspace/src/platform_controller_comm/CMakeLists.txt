cmake_minimum_required(VERSION 3.0.2)
project(platform_controller_comm)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  message_generation
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   MotorStatus.msg
   PacketContent.msg
   WheelRotation.msg
   DebugMsg.msg
)

## Generate services in the 'srv' folder
add_service_files(
 FILES
 LightCommand.srv
 MotorsServices.srv
)

## Generate actions in the 'action' folder
add_action_files(
  FILES
  MotorsAction.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  actionlib_msgs
  sensor_msgs
  std_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES platform_controller_comm
#  CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


#############
## Install ##
#############
set(CMAKE_CXX_STANDARD 17)

install(DIRECTORY urdf/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf
)

add_executable(keyboard_control src/keyboard_control.cpp)
add_dependencies(keyboard_control ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(keyboard_control ${catkin_LIBRARIES})

add_executable(central_hub_cpp src/central_hub_com.cpp)
add_dependencies(central_hub_cpp ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(central_hub_cpp ${catkin_LIBRARIES})

add_executable(front_motor_driver_cpp src/front_motor_com.cpp)
add_dependencies(front_motor_driver_cpp ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(front_motor_driver_cpp ${catkin_LIBRARIES})

add_executable(rear_motor_driver_cpp src/rear_motor_com.cpp)
add_dependencies(rear_motor_driver_cpp ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(rear_motor_driver_cpp ${catkin_LIBRARIES})
