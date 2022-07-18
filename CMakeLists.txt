cmake_minimum_required(VERSION 3.0.2)
project(ruckig_control)

if(NOT "${CMAKE_CXX_STANDARD}")
  set(CMAKE_CXX_STANDARD 11)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  cmake_modules
  control_msgs
  control_toolbox
  controller_manager
  hardware_interface
  joint_limits_interface
  roscpp
  std_msgs
  trajectory_msgs
)

find_package(ruckig REQUIRED)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake)

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    actionlib
    control_msgs
    control_toolbox
    controller_manager
    hardware_interface
    joint_limits_interface
    roscpp
    std_msgs
    trajectory_msgs
  LIBRARIES
    generic_hw_control_loop
    generic_hw_interface
    sim_hw_interface
)

## Build --------------------------------------------------------------

include_directories(
  include/
  ${catkin_INCLUDE_DIRS}
)

set(ruckig_control_SOURCES
  src/ruckig_controller.cpp
)

add_library(RuckigController ${ruckig_control_SOURCES})

target_include_directories(RuckigController PUBLIC
  include
  ${catkin_INCLUDE_DIRS})

target_link_libraries(RuckigController PUBLIC
  ruckig::ruckig
  ${catkin_LIBRARIES})

# Require C++11
set_property(TARGET RuckigController PROPERTY CXX_STANDARD 11)
set_property(TARGET RuckigController PROPERTY CXX_STANDARD_REQUIRED ON)

## Install ------------------------------------------------------------

install(TARGETS RuckigController
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES plugins/custom_controller.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
