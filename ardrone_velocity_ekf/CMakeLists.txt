cmake_minimum_required(VERSION 2.8.3)
project(ardrone_velocity_ekf)
SET(CMAKE_CXX_FLAGS "-std=c++0x")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  geometry_msgs
  nav_msgs
  roscpp
  std_msgs
  tf
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
add_library(pid_control_ekf src/pid_control.cpp)
add_library(filtervelocity src/filtervelocity.cpp)
add_library(coordinate_transform src/coordinate_transform.cpp)
## Declare a cpp executable
add_executable(run_control src/run_control.cpp)
add_executable(run_transform src/run_transform.cpp)

add_dependencies(run_control ${PROJECT_NAME}_gencfg)
add_dependencies(pid_control_ekf ${PROJECT_NAME}_gencfg)
add_dependencies(run_transform  ${PROJECT_NAME}_gencfg)
add_dependencies(coordinate_transform  ${PROJECT_NAME}_gencfg)


## Specify libraries to link a library or executable target against
target_link_libraries(run_control pid_control_ekf ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES})
target_link_libraries(pid_control_ekf filtervelocity ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES})
target_link_libraries(coordinate_transform filtervelocity ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES})
target_link_libraries(run_transform  coordinate_transform  ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES})

