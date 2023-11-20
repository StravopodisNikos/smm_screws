cmake_minimum_required(VERSION 3.0.2)
project(smm_screws)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  tf2
  tf2_ros
  message_generation # [18-10-23] Added for custom msg/srv files generation
  actionlib_msgs     # [ 3-11-23] Added for custom action files generation
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  JointAccel.msg
  CurrentCartesianState.msg
 )

## Generate services in the 'srv' folder
add_service_files(
  FILES
  SetOperationalSpaceJacobian.srv
  SetCurrentCartesianState.srv
)

## Generate actions in the 'action' folder
add_action_files(
  DIRECTORY action
  FILES idosc.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  actionlib_msgs
 )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
#  CATKIN_DEPENDS  
#  LIBRARIES smm_screws
  CATKIN_DEPENDS geometry_msgs roscpp std_msgs tf2 tf2_ros message_runtime  actionlib_msgs # [18-10-23] Added for custom msg generation # [3-11-23] Added action generation
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ~/cpp_ws/libraries/eigen-3.4.0
  ~/cpp_ws/libraries/odeint-v2-master
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
 add_library(ScrewsMain
   src/ScrewsMain.cpp
 )
 add_library(ScrewsKinematics
   src/ScrewsKinematics.cpp
 )
 add_library(ScrewsDynamics
 src/ScrewsDynamics.cpp
)
add_library(robot_shared
src/robot_shared.cpp
)
add_library(OperationalSpaceControllers
src/OperationalSpaceControllers.cpp
)
## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(example src/example.cpp)
add_executable(example_dynamics src/example_dynamics.cpp)
add_executable(example_shared src/example_shared.cpp)
add_executable(serverOperationalSpaceJacobian src/serverOperationalSpaceJacobian.cpp)
add_executable(clientOperationalSpaceJacobian src/clientOperationalSpaceJacobian.cpp)
add_executable(serverCurrentCartesianState src/serverCurrentCartesianState.cpp)
add_executable(clientCurrentCartesianState src/clientCurrentCartesianState.cpp)
add_executable(publishCurrentCartesianState src/publishCurrentCartesianState.cpp)
add_executable(idosc_centralized src/idosc_centralized.cpp)
add_executable(example_idosc_centralized src/example_idosc_centralized.cpp)
## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
## ONLY FOR LIBRARIES THAT HAVE A .cpp FILE!
 target_link_libraries(example
   ${catkin_LIBRARIES}
   # ScrewsMain # -> [4-10-23] Removed since ScrewsKinematics inherits from ScrewsMain
   #ScrewsKinematics
   robot_shared
 )
 target_link_libraries(example_dynamics
   ${catkin_LIBRARIES}
   #ScrewsDynamics
   robot_shared # only test
 ) 
 target_link_libraries(example_shared
 ${catkin_LIBRARIES}
 robot_shared 
) 
target_link_libraries(serverOperationalSpaceJacobian
${catkin_LIBRARIES}
robot_shared 
) 
target_link_libraries(clientOperationalSpaceJacobian
${catkin_LIBRARIES}
robot_shared 
) 
target_link_libraries(serverCurrentCartesianState
${catkin_LIBRARIES}
robot_shared 
) 
target_link_libraries(clientCurrentCartesianState
${catkin_LIBRARIES}
robot_shared 
) 
target_link_libraries(publishCurrentCartesianState
${catkin_LIBRARIES}
robot_shared 
) 
target_link_libraries(idosc_centralized
${catkin_LIBRARIES}
OperationalSpaceControllers 
) 
target_link_libraries(example_idosc_centralized
${catkin_LIBRARIES}
OperationalSpaceControllers 
) 
# [4-10-23] Added Link between libraries, due to cpp inheritance
target_link_libraries(ScrewsKinematics
  ${catkin_LIBRARIES}
  ScrewsMain
)
target_link_libraries(ScrewsDynamics
  ${catkin_LIBRARIES}
  ScrewsKinematics
)
target_link_libraries(robot_shared
  ${catkin_LIBRARIES}
  ScrewsDynamics
)
target_link_libraries(OperationalSpaceControllers
  ${catkin_LIBRARIES}
  robot_shared
)
#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_smm_screws.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
