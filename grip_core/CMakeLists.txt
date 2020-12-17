cmake_minimum_required(VERSION 2.8.3)
project(grip_core)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf2_ros
  sensor_msgs
  trajectory_msgs
  actionlib
  message_generation
  moveit_ros_planning_interface
  geometry_msgs
  moveit_msgs
  actionlib_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

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
  TorqueIntensity.msg
  StandardisedGrasp.msg
  ManipulatorState.msg
  GraspCommand.msg
  RobotPose.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  SetInitACM.srv
  GetModifiedACM.srv
  UpdateACMEntry.srv
  AddStandardisedGrasp.srv
  GetStandardisedGrasp.srv
  AddMoveitPlan.srv
  GetMoveitPlan.srv
  AddJointState.srv
  GetJointState.srv
  AddRobotPose.srv
  GetRobotPose.srv
  AddPoseStamped.srv
  GetPoseStamped.srv
  AddJointTrajectory.srv
  GetJointTrajectory.srv
  AddGazeboMapping.srv
  GetGazeboType.srv
  DeleteGazeboMapping.srv
)

## Generate actions in the 'action' folder
add_action_files(
  FILES
  Grasp.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  trajectory_msgs
  actionlib_msgs
  geometry_msgs
  moveit_msgs
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
 LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS roscpp rospy std_msgs message_runtime sensor_msgs geometry_msgs moveit_msgs actionlib_msgs trajectory_msgs
 DEPENDS YAML_CPP
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDEDIR}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  utils/message_utils.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/grip_core_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

add_executable(moveit_grasp_action_server nodes/moveit_grasp_action_server.cpp)
target_link_libraries(moveit_grasp_action_server ${catkin_LIBRARIES})
add_dependencies(moveit_grasp_action_server grip_core)

add_executable(sensors_tf2_broadcaster nodes/add_sensors_tf2_frame.cpp)
target_link_libraries(sensors_tf2_broadcaster ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})
add_dependencies(sensors_tf2_broadcaster grip_core)

add_executable(acm_manager_server nodes/acm_manager.cpp)
target_link_libraries(acm_manager_server ${catkin_LIBRARIES})
add_dependencies(acm_manager_server grip_core)

add_executable(standardised_grasp_manager_server nodes/standardised_grasp_manager.cpp)
target_link_libraries(standardised_grasp_manager_server ${catkin_LIBRARIES})
add_dependencies(standardised_grasp_manager_server grip_core)

add_executable(moveit_plan_manager_server nodes/moveit_plan_manager.cpp)
target_link_libraries(moveit_plan_manager_server ${catkin_LIBRARIES})
add_dependencies(moveit_plan_manager_server grip_core)

add_executable(joint_state_manager_server nodes/joint_state_manager.cpp)
target_link_libraries(joint_state_manager_server ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})
add_dependencies(joint_state_manager_server grip_core)

add_executable(pose_stamped_manager_server nodes/pose_stamped_manager.cpp)
target_link_libraries(pose_stamped_manager_server ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})
add_dependencies(pose_stamped_manager_server grip_core)

add_executable(joint_trajectory_manager_server nodes/joint_trajectory_manager.cpp)
target_link_libraries(joint_trajectory_manager_server ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})
add_dependencies(joint_trajectory_manager_server grip_core)

add_executable(gazebo_mapping_server nodes/gazebo_mapping.cpp)
target_link_libraries(gazebo_mapping_server ${catkin_LIBRARIES})
add_dependencies(gazebo_mapping_server grip_core)

add_executable(robot_pose_manager_server nodes/robot_pose_manager.cpp)
target_link_libraries(robot_pose_manager_server ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})
add_dependencies(robot_pose_manager_server grip_core)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
)

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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_grip_core.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
