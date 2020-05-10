cmake_minimum_required(VERSION 2.8.3)
project(lbr_iiwa_description)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  ros_topic
  roscpp
  rviz
  tf
  urdf
  xacro
  kdl_parser
  sensor_msgs
)

find_package(Eigen3 REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


################################################
## Declare ROS messages, services and actions ##
################################################

## Generate added messages and services with any dependencies listed here
 ##generate_messages(
   ##DEPENDENCIES
   ##std_msgs  # Or other packages containing msgs
 ##)

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
#  INCLUDE_DIRS include
#  LIBRARIES lbr_iiwa_description
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)


###########
## Build ##
###########

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
	pkg_check_modules(GAZEBO gazebo)
endif()

## Specify additional locations of header files
## Your package locations should be listed before other locations
##include_directories(${Eigen_INCLUDE_DIRS})
include_directories(
      ${catkin_INCLUDE_DIRS}
      ${EIGEN3_INCLUDE_DIR}
      ${GAZEBO_INCLUDE_DIRS}
      )
link_directories(${GAZEBO_LIBRARY_DIRS})

# DECLARE A cpp LIBRARY
add_library(plugin_clik SHARED src/plugin/plugin_clik.cpp)
target_link_libraries(plugin_clik ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${EIGEN3_LIBRARIES})


# Declare a cpp executable 
add_executable( send_pose src/send_pose.cpp)
target_link_libraries(send_pose ${catkin_LIBRARIES})

