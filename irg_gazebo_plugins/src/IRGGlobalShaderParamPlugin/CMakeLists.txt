set(TARGET_NAME IRGGlobalShaderParamPlugin)

set (HEADERS
  GlobalShaderParamPlugin.h
)

set (SOURCES
  GlobalShaderParamPlugin.cpp
)

include_directories(
  ${OGRE_Paging_INCLUDE_DIR}
  ${GAZEBO_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
)

add_library(${TARGET_NAME} SHARED 
  ${HEADERS}
  ${SOURCES}
)

target_link_libraries(${TARGET_NAME}
  ${GAZEBO_libraries}
  ${rclcpp_LIBRARIES}
)
set_target_properties(${TARGET_NAME} PROPERTIES COMPILE_FLAGS "${GAZEBO_CXX_FLAGS}")

## because we're using custom ros message
#add_dependencies(${TARGET_NAME}  ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

rosidl_target_interfaces(${TARGET_NAME} ${PROJECT_NAME} "rosidl_typesupport_cpp")
