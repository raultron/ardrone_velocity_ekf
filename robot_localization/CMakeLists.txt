cmake_minimum_required(VERSION 2.8.3)
project(robot_localization)

set (CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  diagnostic_msgs
  diagnostic_updater
  geographic_msgs
  geometry_msgs
  message_filters
  message_generation
  nav_msgs
  roscpp
  roslint
  sensor_msgs
  std_msgs
  tf
  tf2
  tf2_geometry_msgs
  tf2_ros)

# Attempt to find Eigen using its own CMake module.
# If that fails, fall back to cmake_modules package.
find_package(Eigen3)
if(NOT EIGEN3_FOUND)
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})
endif()

add_definitions(-DEIGEN_NO_DEBUG -DEIGEN_MPL2_ONLY)

roslint_cpp()

###################################
## catkin specific configuration ##
###################################
add_service_files(
  FILES
  SetPose.srv
  SetDatum.srv
)

generate_messages(
  DEPENDENCIES
  geographic_msgs
  geometry_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES 
    ekf
    filter_base
    filter_utilities
    navsat_transform
    ros_filter
    ros_filter_utilities
  CATKIN_DEPENDS
    cmake_modules
    diagnostic_msgs
    diagnostic_updater
    geographic_msgs
    geometry_msgs
    message_filters
    message_runtime
    nav_msgs
    roscpp
    sensor_msgs
    std_msgs
    tf2
    tf2_geometry_msgs
    tf2_ros
  DEPENDS Eigen
)

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

# Library definitions
add_library(filter_utilities src/filter_utilities.cpp)
add_library(filter_base src/filter_base.cpp)
add_library(ekf src/ekf.cpp)
add_library(ros_filter_utilities src/ros_filter_utilities.cpp)
add_library(ros_filter src/ros_filter.cpp)
add_library(navsat_transform src/navsat_transform.cpp)
add_library(PingThread src/PingThread.cpp)


# Dependencies
add_dependencies(filter_base robot_localization_gencpp)
add_dependencies(navsat_transform robot_localization_gencpp)

# Executables
add_executable(ekf_localization_node src/ekf_localization_node.cpp)
add_executable(navsat_transform_node src/navsat_transform_node.cpp)
add_executable(ping_node src/ping_node.cpp)

# Linking
target_link_libraries(ros_filter_utilities ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES})
target_link_libraries(filter_utilities ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES})
target_link_libraries(filter_base filter_utilities  ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES})
target_link_libraries(ekf filter_base ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES})
target_link_libraries(ros_filter ekf ros_filter_utilities  ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES})
target_link_libraries(ekf_localization_node ros_filter ${catkin_LIBRARIES})
target_link_libraries(navsat_transform filter_utilities ros_filter_utilities ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES})
target_link_libraries(navsat_transform_node navsat_transform ${catkin_LIBRARIES})
target_link_libraries(ping_node PingThread ${catkin_LIBRARIES})
#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS
  ekf
  filter_base
  filter_utilities
  navsat_transform
  PingThread
  ros_filter
  ros_filter_utilities
  ekf_localization_node
  navsat_transform_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
  PATTERN ".svn" EXCLUDE)

install(FILES
  LICENSE
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


##################
# FOR QT CREATOR
##################
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/robot_localization/*.hpp)
FILE(GLOB_RECURSE INC_ALL "include/robot_localization/*.hpp")
FILE(GLOB_RECURSE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)
