set(TARGET_NAME IRGLinkTracksPlugin)

find_package(gazebo_ros REQUIRED)

set (HEADERS
  LinkTracksPlugin.h
)

set (SOURCES
  LinkTracksPlugin.cpp
)

add_library(${TARGET_NAME} SHARED
  ${HEADERS}
  ${SOURCES}
)

rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

ament_target_dependencies(${TARGET_NAME}
  gazebo_ros
  geometry_msgs
  rclcpp
  std_msgs
)
target_link_libraries(${TARGET_NAME} ${GAZEBO_LIBRARIES} ${rclcpp_LIBRARIES} ${cpp_typesupport_target})
set_target_properties(${TARGET_NAME} PROPERTIES COMPILE_FLAGS "${GAZEBO_CXX_FLAGS}")

install(TARGETS ${TARGET_NAME}
  LIBRARY DESTINATION lib
)

