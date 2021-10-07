cmake_minimum_required(VERSION 3.0.2)
project(choreonoid_controller_bridge)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  choreonoid
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES choreonoid_controller_bridge
  CATKIN_DEPENDS
  roscpp
  std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CHOREONOID_INCLUDE_DIRS}
)

link_directories(
  ${CHOREONOID_LIBRARY_DIRS}
)

add_library(${PROJECT_NAME} SHARED src/choreonoid_controller_bridge.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${CHOREONOID_BODY_LIBRARIES})

set_target_properties(${PROJECT_NAME} PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CHOREONOID_PLUGIN_SUBDIR}/simplecontroller PREFIX "")
install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION ${CHOREONOID_PLUGIN_SUBDIR}/simplecontroller)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
