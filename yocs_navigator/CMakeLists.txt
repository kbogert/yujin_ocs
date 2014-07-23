cmake_minimum_required(VERSION 2.8.3)
project(yocs_navigator)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  actionlib
  tf
  yocs_math_toolkit
  simple_annotation_msgs
  yocs_msgs
  move_base_msgs
  nav_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES yocs_navigator
  CATKIN_DEPENDS roscpp actionlib tf yocs_math_toolkit simple_annotation_msgs move_base_msgs nav_msgs yocs_msgs 
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Declare a cpp library
add_library(yocs_basic_move_controller
  src/${PROJECT_NAME}/basic_move_controller.cpp
)

target_link_libraries(yocs_basic_move_controller ${catkin_LIBRARIES})
add_dependencies(yocs_basic_move_controller geometry_msgs_gencpp nav_msgs_gencpp)

## Declare a cpp executable
# add_executable(yocs_navigator_node src/yocs_navigator_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(yocs_navigator_node yocs_navigator_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(yocs_navigator_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

install(TARGETS yocs_basic_move_controller
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)