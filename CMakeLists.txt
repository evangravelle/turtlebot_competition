cmake_minimum_required(VERSION 2.8.3)
project(coconuts_odroid)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  geometry_msgs
  roscpp
  rospy
  serial
  std_msgs
  tf
  cv_bridge
  image_transport
  coconuts_common
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
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
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
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   DIRECTORY action
#   FILES MoveMotor.action
# )

## Generate added messages and services with any dependencies listed here
#generate_messages(
#  DEPENDENCIES actionlib_msgs geometry_msgs std_msgs
#)
generate_messages()
################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
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
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES coconuts_odroid
#  CATKIN_DEPENDS actionlib actionlib_msgs geometry_msgs roscpp rospy serial std_msgs tf cv_bridge image_transport
#  DEPENDS system_lib
CATKIN_DEPENDS message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(coconuts_odroid
#   src/${PROJECT_NAME}/coconuts_odroid.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
## add_dependencies(Bridge ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

  catkin_install_python(PROGRAMS
    src/globalLocalizationPY.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

## Declare a C++ executable
add_executable(Bridge src/Bridge.cpp)
add_executable(arm_control src/arm_control.cpp)
add_executable(arm_poll src/arm_poll.cpp)
add_executable(MockArduino src/MockArduino.cpp)
add_executable(detect_objects_forward src/detect_objects_forward.cpp)
add_executable(detect_ball_down src/detect_ball_down.cpp)
add_executable(detect_ball_left src/detect_ball_left.cpp)
add_executable(detect_ball_right src/detect_ball_right.cpp)
add_executable(publishInfo src/publishInfo.cpp)
add_executable(getDistance src/getDistance.cpp)
add_executable(locate_ball_forward src/locate_ball_forward.cpp)
add_executable(locate_ball_down src/locate_ball_forward.cpp)
#add_executable(globalLocalization src/globalLocalization.cpp)
#add_executable(globalLocalization src/globalLocalization.py)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(Bridge ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} coconuts_common_generate_messages_cpp)
add_dependencies(arm_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} coconuts_common_generate_messages_cpp)
add_dependencies(arm_poll ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} coconuts_common_generate_messages_cpp)
add_dependencies(MockArduino ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(detect_objects_forward ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(detect_ball_down ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(detect_ball_left ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(detect_ball_right ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(publishInfo ${catkin_EXPORTED_TARGETS})
add_dependencies(getDistance ${catkin_EXPORTED_TARGETS})
add_dependencies(locate_ball_forward ${catkin_EXPORTED_TARGETS})
add_dependencies(locate_ball_down ${catkin_EXPORTED_TARGETS})
#add_dependencies(globalLocalization ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(Bridge ${catkin_LIBRARIES})
target_link_libraries(arm_control ${catkin_LIBRARIES})
target_link_libraries(arm_poll ${catkin_LIBRARIES})
target_link_libraries(MockArduino ${catkin_LIBRARIES})
target_link_libraries(detect_objects_forward ${catkin_LIBRARIES})
target_link_libraries(detect_ball_down ${catkin_LIBRARIES})
target_link_libraries(detect_ball_left ${catkin_LIBRARIES})
target_link_libraries(detect_ball_right ${catkin_LIBRARIES})
target_link_libraries(publishInfo ${catkin_LIBRARIES})
target_link_libraries(getDistance ${catkin_LIBRARIES})
target_link_libraries(locate_ball_forward ${catkin_LIBRARIES})
target_link_libraries(locate_ball_down ${catkin_LIBRARIES})
#target_link_libraries(globalLocalization ${catkin_LIBRARIES})
#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination


## Mark executables and/or libraries for installation
# install(TARGETS coconuts_odroid coconuts_odroid_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_coconuts_odroid.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
