cmake_minimum_required(VERSION 3.8)
project(smm_screws)

# Use C++17 (you already use it in your manual g++ commands)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

#######################################
## Find packages (AMENT + ROS 2)     ##
#######################################

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rosidl_default_generators REQUIRED)

#######################################
## Interface definitions (ROS 2)     ##
#######################################

set(msg_files
  "msg/JointAccel.msg"
  "msg/CurrentCartesianState.msg"
  "msg/ForceMeasurements.msg"
  "msg/JointVel.msg"
)

set(srv_files
  "srv/SetOperationalSpaceJacobian.srv"
  "srv/SetCurrentCartesianState.srv"
)

set(action_files
  "action/Idosc.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES std_msgs geometry_msgs
)

# Export runtime dependency for generated interfaces
ament_export_dependencies(rosidl_default_runtime)

#######################################
## Include directories               ##
#######################################

# Core headers are under include/smm_screws/core
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  # If you really need these custom libs, keep them. Otherwise comment them out.
  # /home/nikos/cpp_ws/libraries/odeint-v2-master
  # /home/nikos/cpp_ws/libraries/fuzzylite/include
)

#######################################
## Optional: fuzzylite imported lib  ##
#######################################

# If you still use fuzzylite and the path is valid, keep this.
# Otherwise comment this whole block.
#add_library(fuzzylite_lib SHARED IMPORTED)
#set_target_properties(fuzzylite_lib PROPERTIES
#  IMPORTED_LOCATION /home/nikos/cpp_ws/libraries/fuzzylite/build/libfuzzylite.so
#)

#######################################
## Core C++ libraries (no ROS nodes) ##
#######################################

# ScrewsMain
add_library(ScrewsMain
  src/core/ScrewsMain.cpp
)
target_include_directories(ScrewsMain PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(ScrewsMain
  Eigen3::Eigen
)

# ScrewsKinematics
add_library(ScrewsKinematics
  src/core/ScrewsKinematics.cpp
)
target_include_directories(ScrewsKinematics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(ScrewsKinematics
  ScrewsMain
  Eigen3::Eigen
)

# ScrewsKinematicsNdof
add_library(ScrewsKinematicsNdof
  src/core/ScrewsKinematicsNdof.cpp
)

target_include_directories(ScrewsKinematicsNdof PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(ScrewsKinematicsNdof
  ScrewsMain
  Eigen3::Eigen
)

# ScrewsDynamics
add_library(ScrewsDynamics
  src/core/ScrewsDynamics.cpp
)
target_include_directories(ScrewsDynamics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(ScrewsDynamics
  ScrewsKinematics
  Eigen3::Eigen
  yaml-cpp
)

# RobotYamlLoader
add_library(robot_yaml_loader
  src/core/RobotYamlLoader.cpp
)
target_include_directories(robot_yaml_loader PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(robot_yaml_loader
  yaml-cpp
  Eigen3::Eigen
)

# RobotYamlLoaderNdof
add_library(robot_yaml_loader_ndof
  src/core/RobotYamlLoaderNdof.cpp
)
target_include_directories(robot_yaml_loader_ndof PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(robot_yaml_loader_ndof
  yaml-cpp
  Eigen3::Eigen
)

# RobotContext
add_library(RobotContextLib
  src/core/RobotContext.cpp
)

target_include_directories(RobotContextLib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(RobotContextLib PUBLIC
  ScrewsKinematics
  ScrewsDynamics
  robot_yaml_loader
  yaml-cpp
  Eigen3::Eigen
)

# RobotContextNdof
add_library(RobotContextLibNdof
  src/core/RobotContextNdof.cpp
)

target_include_directories(RobotContextLibNdof PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(RobotContextLibNdof PUBLIC
  ScrewsKinematicsNdof
  #ScrewsDynamics
  robot_yaml_loader_ndof
  yaml-cpp
  Eigen3::Eigen
)

#######################################
## Simple core test executable       ##
#######################################

# Your pure C++ test (no ROS)
add_executable(test_screws_core
  src/core/test_screws_core.cpp
)
target_link_libraries(test_screws_core
  ScrewsDynamics
  robot_yaml_loader
)

#######################################
## Installation                      ##
#######################################

# Install headers
install(
  DIRECTORY include/
  DESTINATION include
)

# Install libraries and test binary
install(
  TARGETS
    ScrewsMain
    ScrewsKinematics
    ScrewsKinematicsNdof
    ScrewsDynamics
    robot_yaml_loader
    robot_yaml_loader_ndof
    RobotContextLib
    RobotContextLibNdof
    test_screws_core
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# -----------------------------------------
# Examples: pure C++ (no ROS)
# -----------------------------------------
option(SMM_SCREWS_BUILD_EXAMPLES "Build C++ core examples" ON)

if(SMM_SCREWS_BUILD_EXAMPLES)
# ---------- Simple core C++ example: screws_main_example ----------

add_executable(smm_screws_main_example
  examples/cpp_core/screws_main_example.cpp
)

target_include_directories(smm_screws_main_example PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(smm_screws_main_example
  PUBLIC
    ScrewsMain         # your core library from src/core/ScrewsMain.cpp
    Eigen3::Eigen
)

install(
  TARGETS smm_screws_main_example
  DESTINATION lib/${PROJECT_NAME}
)
endif()

# -----------------------------------------
# Demos: ROS 2 nodes
# -----------------------------------------
option(SMM_SCREWS_BUILD_DEMOS "Build ROS 2 demo nodes" ON)

if(SMM_SCREWS_BUILD_DEMOS)
  add_executable(smm_fk_demo_node
    demos/ros2/fk_demo_node.cpp
  )

  target_include_directories(smm_fk_demo_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${smm_screws_INCLUDE_DIRS}
  )

  target_link_libraries(smm_fk_demo_node
    PUBLIC
      rclcpp::rclcpp
      Eigen3::Eigen
      yaml-cpp
      ScrewsKinematics
      ScrewsDynamics
      RobotContextLib
  )

  install(
    TARGETS smm_fk_demo_node
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

#######################################
## Export                            ##
#######################################

ament_export_include_directories(include)
ament_export_libraries(
  ScrewsMain
  ScrewsKinematics
  ScrewsKinematicsNdof
  ScrewsDynamics
  robot_yaml_loader
  robot_yaml_loader_ndof
  RobotContextLib
  RobotContextLibNdof
)

ament_export_dependencies(
  rclcpp
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  yaml-cpp
  rosidl_default_runtime
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()
