cmake_minimum_required(VERSION 2.8.3)

# wheel motor raw data is published directly to ROS
# add_definitions(-D WHEEL_ODOMETRY_ROS)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  roslaunch 
  roscpp
  rospy
  std_msgs
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mecanum_robot
#  CATKIN_DEPENDS message_runtime roscpp std_msgs robot_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(.)
include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

set(CMAKE_CXX_FLAGS "-std=c++14")

## Declare a C++ library
# add_library(mecanum_robot
#   src/${PROJECT_NAME}/mecanum_robot.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure

## Declare a C++ executable
add_executable(area_control
  area_control.cpp
)
##add_dependencies(area_control robot_msgs_gencpp)
add_dependencies(area_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})



## Mark executables and/or libraries for installation
install(TARGETS area_control
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
target_link_libraries(area_control
  ${catkin_LIBRARIES}
)



