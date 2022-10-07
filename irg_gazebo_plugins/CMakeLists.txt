cmake_minimum_required(VERSION 3.5.0)
project(irg_gazebo_plugins)

set(CMAKE_CXX_STANDARD 14)
add_compile_options(-Wall -Wpedantic -Wno-unused-but-set-parameter -Wno-register)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(OGRE REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Set GL preference to quiet build warning
set(OpenGL_GL_PREFERENCE GLVND)
find_package(OpenGL REQUIRED)

message(STATUS "FOUND QT 5")
find_package(Qt5Widgets REQUIRED)
find_package(Qt5Xml REQUIRED)
if (Qt5_POSITION_INDEPENDENT_CODE)
  SET(CMAKE_POSITION_INDEPENDENT_CODE ON)
endif()
include_directories(SYSTEM "${Qt5Widgets_INCLUDE_DIRS}")
include_directories(SYSTEM "${Qt5Xml_INCLUDE_DIRS}")
set(QT_QTXML_LIBRARY Qt5::Xml)
set(QT_QTCORE_LIBRARY Qt5::Widgets)

include_directories(
  ${GAZEBO_INCLUDE_DIRS}
)

option(ENABLE_PROFILER "Enable/disable Ignition Profiler" OFF)
if(ENABLE_PROFILER)
  find_package(ignition-common3 COMPONENTS profiler)
  if(ignition-common3_FOUND)
    add_definitions("-DIGN_PROFILER_ENABLE=1" "-DIGN_PROFILER_REMOTERY=1")
    message(STATUS "Profiler is active")
  else()
    message(WARNING "Can't find Ignition common3. Profiler will not be actived")
  endif()
endif()

# mallan: add plugins env
ament_environment_hooks(${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/65.irg_gazebo_plugins.dsv.in)

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

# Generate messages in the 'msg' folder
rosidl_generate_interfaces(${PROJECT_NAME}
  msg/ShaderParamUpdate.msg
  DEPENDENCIES
  std_msgs
)


## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )


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
#catkin_package(
##  INCLUDE_DIRS include
##  LIBRARIES irg_gazebo_plugins
#  CATKIN_DEPENDS roscpp std_msgs tf2 tf2_ros
##  DEPENDS system_lib
#)
ament_export_dependencies(rclcpp std_msgs rosidl_default_generators rosidl_default_runtime tf2 tf2_ros)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)


add_subdirectory(src)

install(
  DIRECTORY materials
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
