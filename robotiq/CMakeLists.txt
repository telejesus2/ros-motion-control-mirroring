cmake_minimum_required(VERSION 2.8.3)
project(robotiq)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  ur_modern_driver
  ur_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES gripper_ur_control
  CATKIN_DEPENDS roscpp rospy ur_modern_driver ur_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_library(gripper_ur_control
	src/gripper_ur_control.cpp
)
target_link_libraries(gripper_ur_control ${catkin_LIBRARIES})

install(TARGETS gripper_ur_control
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
   	   PATTERN ".svn" EXCLUDE
   )

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(sample_node src/sample_node.cpp)
target_link_libraries(sample_node ${catkin_LIBRARIES} gripper_ur_control)

add_executable(gripper_fake_joint_states src/fake_joint_states.cpp)
target_link_libraries(gripper_fake_joint_states ${catkin_LIBRARIES})
