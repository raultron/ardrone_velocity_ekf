cmake_minimum_required(VERSION 2.8.3)
project(velocity_control)
SET(CMAKE_CXX_FLAGS "-std=c++0x")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  tf2
)

find_package(Eigen3)
if(NOT EIGEN3_FOUND)
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})
endif()

## Generate dynamic parameters
generate_dynamic_reconfigure_options(
  cfg/dynamic_param_config.cfg
)

catkin_package(
  INCLUDE_DIRS 
  include
  CATKIN_DEPENDS
  dynamic_reconfigure
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  tf2
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

## Add library
add_library(pid_control src/pid_control.cpp)
add_library(mpid_control src/mpid_control.cpp)
add_library(pi_pd_control src/pi_pd_control.cpp)
add_library(filtervelocity src/filtervelocity.cpp)
## Declare a cpp executable
add_executable(run_control src/run_control.cpp)

add_dependencies(run_control ${PROJECT_NAME}_gencfg)
add_dependencies(pid_control ${PROJECT_NAME}_gencfg)


## Specify libraries to link a library or executable target against
target_link_libraries(run_control pid_control ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES})
target_link_libraries(run_control mpid_control ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES})
target_link_libraries(run_control pi_pd_control ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES})
target_link_libraries(pid_control filtervelocity ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES})
target_link_libraries(pi_pd_control filtervelocity ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES})

##################
# FOR QT CREATOR
##################
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/*.h)
FILE(GLOB_RECURSE INC_ALL "include/coop_uv_qt/*.hpp")
FILE(GLOB_RECURSE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)

