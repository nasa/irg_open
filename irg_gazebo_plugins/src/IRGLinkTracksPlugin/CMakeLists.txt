set(TARGET_NAME IRGLinkTracksPlugin)

find_package(gazebo_ros REQUIRED)

set (HEADERS
  LinkTracksPlugin.h
)

set (SOURCES
  LinkTracksPlugin.cpp
)

include_directories(
  ${rclcpp_INCLUDE_DIRS}
  ${OGRE_Paging_INCLUDE_DIR}
  ${GAZEBO_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
)

add_library(${TARGET_NAME} SHARED 
  ${HEADERS}
  ${SOURCES}
)

ament_target_dependencies(${TARGET_NAME} "gazebo_ros")
target_link_libraries(${TARGET_NAME} ${GAZEBO_libraries} ${rclcpp_LIBRARIES})
set_target_properties(${TARGET_NAME} PROPERTIES COMPILE_FLAGS "${GAZEBO_CXX_FLAGS}")

rosidl_target_interfaces(${TARGET_NAME} ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS ${TARGET_NAME}
  LIBRARY DESTINATION lib
)
