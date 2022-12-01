set(TARGET_NAME IRGGlobalShaderParamPlugin)

find_package(gazebo_ros REQUIRED)

set (HEADERS
  GlobalShaderParamPlugin.h
)

set (SOURCES
  GlobalShaderParamPlugin.cpp
)

add_library(${TARGET_NAME} SHARED 
  ${HEADERS}
  ${SOURCES}
)

rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

target_link_libraries(${TARGET_NAME}
  ${GAZEBO_LIBRARIES}
  ${cpp_typesupport_target}
)

ament_target_dependencies(${TARGET_NAME} gazebo_ros rclcpp std_msgs)
set_target_properties(${TARGET_NAME} PROPERTIES COMPILE_FLAGS "${GAZEBO_CXX_FLAGS}")

install(TARGETS ${TARGET_NAME}
  LIBRARY DESTINATION lib
)
