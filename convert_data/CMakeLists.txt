cmake_minimum_required(VERSION 2.8.3)
project(convert_data)

find_package(catkin REQUIRED COMPONENTS
  ardrone_autonomy
  geographic_msgs
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

catkin_package(
  INCLUDE_DIRS
  include
  CATKIN_DEPENDS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  tf2
)

include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})

# Build the executable
add_executable(convert_nav src/convert_nav.cpp)
add_executable(vrep_connection src/vrep_connection.cpp)
# Add a build order dependency on nav_msgs
# This ensures that nav_msgs' msg headers are built before your executable
if(nav_msgs_EXPORTED_TARGETS)
  add_dependencies(convert_nav ${nav_msgs_EXPORTED_TARGETS})
endif()
# Link against the catkin libraries
target_link_libraries(convert_nav ${catkin_LIBRARIES})
target_link_libraries(vrep_connection ${catkin_LIBRARIES})
# Install the executable
install(TARGETS convert_nav
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


##################
# FOR QT CREATOR
##################
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/*.h)
FILE(GLOB_RECURSE INC_ALL "include/coop_uv_qt/*.hpp")
FILE(GLOB_RECURSE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)
